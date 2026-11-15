//! PLC-style sequence authoring: a sequence is a list of *steps* (工程),
//! each firing entry actions and waiting on a transition condition — the
//! SFC / step-ladder mental model. The robot is one device among several:
//! motions are *started* by an action and *awaited* by a condition.
//!
//! Vocabulary mapping (see design/design-sequence-control.md §3):
//! internal signals = internal relays (M), `Elapsed` = an on-delay timer
//! (TON), `Done` = the robot's completion signal, the scan loop lives in
//! [`crate::rollout`].

use nalgebra::{Isometry3, Point3, Unit, Vector3};

use crate::{Scene, SceneError};

/// A user-defined internal signal (PLC internal relay), written by
/// [`Action::Set`] and read by [`Condition::Signal`].
#[derive(Debug, Clone)]
pub struct SignalDef {
    pub name: String,
    pub initial: bool,
}

/// A user-defined internal word (a PLC data register): a number programs
/// compare ([`Condition::Compare`]) and write ([`Action::SetWord`], a
/// rung's [`RungOutput::Move`]) — a setpoint, a computed value, a process
/// quantity a plant-side program integrates (design-plc-resolution.md
/// PF2). Recorded as a word lane under its own name.
#[derive(Debug, Clone, PartialEq)]
pub struct WordDef {
    pub name: String,
    pub initial: f64,
    /// An `INT` (counts, a selector) rather than a `REAL` measurement:
    /// every write is rounded to the nearest whole number.
    pub integer: bool,
}

/// An analog input: a transmitter on a 4–20 mA loop into an input module.
/// It measures `source` — a word lane (a defined word, a device's
/// encoder) — over `range` (engineering units at 4 and 20 mA); the module
/// converts the current to counts over `raw`; the controller scales the
/// counts back. Every scan the plant records the counts as the word lane
/// `<name>.raw` (the value on the wire, an `INT`) and the scaled value a
/// program reads as the word `<name>` — what the controller computes from
/// the counts, so the quantisation is in the bake too.
///
/// An open wire ([`FaultKind::Open`]) reads 0 mA: the counts fall below
/// `raw.0`, which is what a wire-break rung compares against.
#[derive(Debug, Clone, PartialEq)]
pub struct Analog {
    pub name: String,
    /// The word lane measured.
    pub source: String,
    /// Engineering value at 4 mA and at 20 mA.
    pub range: (f64, f64),
    /// The engineering unit (`"kPa"`, `"%"`), for the tables.
    pub unit: String,
    /// The input module's counts at 4 mA and at 20 mA (`(0, 4000)`).
    pub raw: (i64, i64),
    /// The obstacle that is the transmitter's body, when it has one —
    /// where the studio hangs the input's tag. Display only.
    pub place: Option<String>,
}

/// The current a transmitter sends past its range before it saturates
/// (NAMUR NE 43's measuring band): 3.8 … 20.5 mA.
pub const ANALOG_MA: (f64, f64) = (3.8, 20.5);

impl Analog {
    /// The loop current for an engineering value: linear over the range,
    /// saturating at [`ANALOG_MA`]. `None` (an open wire) is 0 mA.
    pub fn current(&self, value: Option<f64>) -> f64 {
        match value {
            None => 0.0,
            Some(x) => {
                let span = self.range.1 - self.range.0;
                (4.0 + 16.0 * (x - self.range.0) / span).clamp(ANALOG_MA.0, ANALOG_MA.1)
            }
        }
    }

    /// The module's counts for a loop current.
    pub fn counts(&self, ma: f64) -> i64 {
        let span = (self.raw.1 - self.raw.0) as f64;
        (self.raw.0 as f64 + (ma - 4.0) * span / 16.0).round() as i64
    }

    /// The engineering value the controller scales the counts back to —
    /// the same expression, in the same order, the ST export writes.
    pub fn scaled(&self, counts: i64) -> f64 {
        self.range.0
            + (counts - self.raw.0) as f64 * (self.range.1 - self.range.0)
                / (self.raw.1 - self.raw.0) as f64
    }
}

/// One side of a comparison or of a word expression: a word by name, or
/// a constant.
#[derive(Debug, Clone, PartialEq)]
pub enum Operand {
    /// A word lane: a defined word, an analog input (`lvl`, or its counts
    /// `lvl.raw`), a device's `<device>.encoder`, a reader's
    /// `<sensor>.id`, a counter's `<signal>.count`.
    Word(String),
    Const(f64),
}

/// A number as the tables print it: whole numbers without a fraction.
pub fn word_text(x: f64) -> String {
    if x.fract() == 0.0 && x.abs() < 1e15 {
        format!("{}", x as i64)
    } else {
        format!("{x}")
    }
}

impl std::fmt::Display for Operand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Operand::Word(name) => write!(f, "{name}"),
            Operand::Const(x) => write!(f, "{}", word_text(*x)),
        }
    }
}

impl std::fmt::Display for WordExpr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.op {
            None => write!(f, "{}", self.left),
            Some((op, right)) => write!(f, "{} {} {right}", self.left, op.as_str()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompareOp {
    Ge,
    Gt,
    Le,
    Lt,
    Eq,
    Ne,
}

impl CompareOp {
    pub fn as_str(self) -> &'static str {
        match self {
            CompareOp::Ge => ">=",
            CompareOp::Gt => ">",
            CompareOp::Le => "<=",
            CompareOp::Lt => "<",
            CompareOp::Eq => "==",
            CompareOp::Ne => "!=",
        }
    }

    pub fn parse(s: &str) -> Option<CompareOp> {
        Some(match s {
            ">=" => CompareOp::Ge,
            ">" => CompareOp::Gt,
            "<=" => CompareOp::Le,
            "<" => CompareOp::Lt,
            "==" => CompareOp::Eq,
            "!=" => CompareOp::Ne,
            _ => return None,
        })
    }

    /// The operator as Structured Text spells it.
    pub fn st(self) -> &'static str {
        match self {
            CompareOp::Eq => "=",
            CompareOp::Ne => "<>",
            other => other.as_str(),
        }
    }

    pub fn holds(self, a: f64, b: f64) -> bool {
        match self {
            CompareOp::Ge => a >= b,
            CompareOp::Gt => a > b,
            CompareOp::Le => a <= b,
            CompareOp::Lt => a < b,
            CompareOp::Eq => a == b,
            CompareOp::Ne => a != b,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArithOp {
    Add,
    Sub,
    Mul,
    Div,
}

impl ArithOp {
    pub fn as_str(self) -> &'static str {
        match self {
            ArithOp::Add => "+",
            ArithOp::Sub => "-",
            ArithOp::Mul => "*",
            ArithOp::Div => "/",
        }
    }

    pub fn parse(s: &str) -> Option<ArithOp> {
        Some(match s {
            "+" => ArithOp::Add,
            "-" => ArithOp::Sub,
            "*" => ArithOp::Mul,
            "/" => ArithOp::Div,
            _ => return None,
        })
    }
}

/// What a word is written with: an operand (a transfer, `MOV`), or one
/// arithmetic instruction over two (`+ - * /`). No nesting — a longer
/// formula is a rung per instruction, as on a PLC.
///
/// An expression over two whole operands (integer words, whole constants)
/// is integer arithmetic — the division truncates toward zero, as IEC
/// `INT / INT` does; any other is `f64` arithmetic. A write to an integer
/// word rounds to the nearest whole number (halves away from zero).
#[derive(Debug, Clone, PartialEq)]
pub struct WordExpr {
    pub left: Operand,
    pub op: Option<(ArithOp, Operand)>,
}

impl WordExpr {
    pub fn constant(value: f64) -> WordExpr {
        WordExpr {
            left: Operand::Const(value),
            op: None,
        }
    }

    /// The operands, left then right.
    pub fn operands(&self) -> impl Iterator<Item = &Operand> {
        std::iter::once(&self.left).chain(self.op.iter().map(|(_, r)| r))
    }

    /// The word lanes the expression reads.
    pub fn words(&self) -> impl Iterator<Item = &str> {
        self.operands().filter_map(|o| match o {
            Operand::Word(name) => Some(name.as_str()),
            Operand::Const(_) => None,
        })
    }
}

/// What a word name turned out to be ([`Scene::word_kind`]).
#[derive(Debug, Clone, PartialEq)]
pub enum WordKind {
    /// A defined word.
    Defined { integer: bool },
    /// An analog input's scaled value.
    Analog,
    /// An analog input's counts (`<name>.raw`).
    AnalogRaw { analog: String },
    /// A device's encoder (`<device>.encoder`).
    Encoder { device: String },
    /// An identification reader's word (`<sensor>.id`).
    Id { sensor: String },
    /// A counter's count (`<signal>.count`).
    Count { signal: String },
}

impl WordKind {
    /// Whether the word holds whole numbers (an `INT`).
    pub fn integer(&self) -> bool {
        match self {
            WordKind::Defined { integer } => *integer,
            WordKind::Analog | WordKind::Encoder { .. } => false,
            WordKind::AnalogRaw { .. } | WordKind::Id { .. } | WordKind::Count { .. } => true,
        }
    }
}

/// A pseudo-sensor, evaluated geometrically each scan tick and published
/// as a read-only *input* signal under its own name (PLC input contact).
#[derive(Debug, Clone)]
pub struct Sensor {
    pub name: String,
    pub kind: SensorKind,
    pub watch: SensorWatch,
    /// Vehicle this sensor rides on: its `kind` geometry is then read in
    /// that vehicle's frame and re-resolved every scan tick (a load-present
    /// eye on a deck, or a protective field that travels with the machine).
    /// `None` — the usual case — is a fixture bolted to the floor.
    pub mount: Option<String>,
    /// An identification reader (RFID, code reader): the identifier of
    /// each watched obstacle, `(obstacle, id)`. Besides its presence lane
    /// the sensor then records a word lane `<name>.id` — the smallest id
    /// among the watched bodies it sees, `0` when it sees none — for a
    /// controller to branch on. `None` is a plain presence sensor.
    pub ids: Option<Vec<(String, i64)>>,
}

#[derive(Debug, Clone)]
pub enum SensorKind {
    /// Presence/area sensor: ON while a watched body overlaps the box.
    Zone {
        pose: Isometry3<f64>,
        size: Vector3<f64>,
    },
    /// Photoelectric beam: ON while the beam segment is interrupted.
    Beam {
        from: Point3<f64>,
        to: Point3<f64>,
        radius: f64,
    },
    /// Vision presence: ON while a watched body overlaps the referenced
    /// camera's view frustum (and, with `occlusion`, its origin is not
    /// hidden behind another obstacle). The camera is the optics — pose,
    /// mount, fov, aspect all come from it; the sensor is the judgement.
    /// Geometry only: no pixels are rendered or interpreted.
    Vision {
        /// The [`Camera`] this sensor looks through.
        camera: String,
        /// Detection band along the view axis, meters; `None` uses the
        /// camera's near/far clip.
        detect_range: Option<[f64; 2]>,
        /// Ray-test the candidate's origin against the *other* obstacles
        /// (one ray — coarse but deterministic). Robot links neither
        /// occlude nor get occlusion-tested; they detect by overlap alone.
        occlusion: bool,
    },
    /// Laser-scanner field: ON while a watched body crosses the
    /// referenced lidar's scan-plane sector (and, with `shadowing`, its
    /// origin is not hidden behind another obstacle). The lidar is the
    /// sweep — pose, mount, fov all come from it; the sensor is the
    /// judgement, so one scanner carries several fields (warning +
    /// protective), the real field-set shape. Geometry only: no per-angle
    /// rays (design/design-lidar.md 判断 L3).
    Field {
        /// The [`Lidar`] this field sweeps through.
        lidar: String,
        /// Field radius, meters; `None` uses the lidar's max range.
        range: Option<f64>,
        /// Angular window `[start, end]` in the scan frame, degrees
        /// (0 = +X, CCW), within the lidar's sweep; `None` uses the full
        /// sweep.
        sector: Option<[f64; 2]>,
        /// Ray-test the candidate's origin against the *other* obstacles
        /// (one ray — coarse but deterministic, the vision rule). Robot
        /// links neither shadow nor get shadow-tested; a vehicle-mounted
        /// field also ignores its own machine's body (the massing chassis
        /// would otherwise blind or trip it).
        shadowing: bool,
    },
}

#[derive(Debug, Clone)]
pub enum SensorWatch {
    /// Only the named obstacles trip the sensor.
    Objects(Vec<String>),
    AllObjects,
    /// Any robot's links trip it (light-curtain style).
    Robot,
    /// Only the named robot instances' links trip it (interlock zones that
    /// must see one arm but not the other).
    Robots(Vec<String>),
    /// Only the named arms' links trip it: `(robot, group)` pairs — the
    /// interlock zone that sees one arm of a dual-arm robot and not the
    /// other.
    Groups(Vec<(String, String)>),
    All,
}

/// A camera: a named viewpoint with pinhole optics, drawn as a frustum in
/// the studio and (in later phases) rendered picture-in-picture, exported
/// as video and as a UsdGeomCamera. Pure presentation in this phase — it
/// publishes no signal and never affects collision, planning or the
/// rollout. A vision *sensor* referencing one is the planned signal path
/// (design/design-camera.md §3): the camera is the optics, a sensor is
/// the judgement.
#[derive(Debug, Clone)]
pub struct Camera {
    pub name: String,
    /// Draw Studio's generic housing; false when a separate visual represents it.
    pub body_visible: bool,
    /// What the camera is bolted to; [`Camera::pose`] is expressed in
    /// this frame.
    pub mount: CameraMount,
    /// Offset in the mount frame (the world pose for
    /// [`CameraMount::World`]). Convention: -Z is the view direction,
    /// +Y is image-up — the three.js / USD camera convention, so both
    /// consumers apply it verbatim.
    pub pose: Isometry3<f64>,
    /// Horizontal field of view, degrees.
    pub fov_deg: f64,
    /// Image size in pixels: the frustum's aspect ratio and the pixel
    /// size of exports.
    pub resolution: [u32; 2],
    /// Near clip distance, meters.
    pub near: f64,
    /// Far clip distance, meters.
    pub far: f64,
}

/// What a [`Camera`] is bolted to.
#[derive(Debug, Clone, PartialEq)]
pub enum CameraMount {
    /// A fixture: the pose is a world pose.
    World,
    /// Rides a vehicle device; re-resolved as the vehicle moves.
    Vehicle { device: String },
    /// Bolted to a robot link (a wrist camera).
    Link { robot: String, link: String },
}

/// A LiDAR scanner: a named scan origin with a planar sweep, drawn as a
/// sector in the studio. Pure presentation in this phase — it publishes
/// no signal and never affects collision, planning or the rollout. A
/// *field* sensor referencing one is the planned signal path
/// (design/design-lidar.md §3): the scanner is the sweep, a sensor is
/// the judgement.
#[derive(Debug, Clone)]
pub struct Lidar {
    pub name: String,
    /// Draw Studio's generic housing; false when a separate visual represents it.
    pub body_visible: bool,
    /// What the scanner is bolted to; [`Lidar::pose`] is expressed in
    /// this frame.
    pub mount: LidarMount,
    /// Offset in the mount frame (the world pose for
    /// [`LidarMount::World`]). Convention: the scan plane is local XY,
    /// angle 0 along +X, counter-clockwise toward +Y — the ROS laser
    /// frame, so catalog devices import with no rotation fix.
    pub pose: Isometry3<f64>,
    /// Full scan angle, degrees, up to 360.
    pub fov_deg: f64,
    /// Measuring range `[min, max]`, meters.
    pub range: [f64; 2],
    /// Angular resolution, degrees — the default step of the scan API.
    /// Presentation/analysis data; field judgement never reads it.
    pub resolution_deg: f64,
    /// Vertical channels (scan rings). `1` is a planar scanner; a 3D
    /// scanner spreads this many rings evenly across [`Lidar::vfov_deg`].
    /// Scan-API data like `resolution_deg` — field judgement stays on
    /// the scan plane either way (real safety fields are 2D;
    /// design/design-lidar.md 判断 L8).
    pub channels: u32,
    /// Full vertical field of view, degrees, centered on the scan plane.
    /// `0` for a planar scanner; meaningful only with `channels > 1`.
    pub vfov_deg: f64,
}

/// What a [`Lidar`] is bolted to.
#[derive(Debug, Clone, PartialEq)]
pub enum LidarMount {
    /// A fixture: the pose is a world pose.
    World,
    /// Rides a vehicle device; re-resolved as the vehicle moves.
    Vehicle { device: String },
    /// Bolted to a robot link.
    Link { robot: String, link: String },
}

/// A weld-current indicator: while `signal` is true, the studio draws an
/// arc flash at `robot`'s TCP and the USD export blinks an emissive prim
/// there. Pure presentation — deterministic (it renders a baked signal),
/// never part of collision or planning. The signal is the PLC-honest
/// driver: a real weld controller's "current on" output, authored by the
/// program that owns the weld.
#[derive(Debug, Clone)]
pub struct WeldFlash {
    pub name: String,
    pub signal: String,
    pub robot: String,
    pub kind: FlashKind,
    /// Link spun visually while the signal is on (a cutter); display
    /// only, never kinematics.
    pub spin_link: Option<String>,
    /// The spray cone's size for [`FlashKind::Spray`]: length along the
    /// TCP's -Z (the standoff, roughly) and base radius (the pattern's).
    pub cone: Option<SprayCone>,
    /// The obstacle a [`FlashKind::Lamp`] lights.
    pub obstacle: Option<String>,
    /// The lamp's lit colour, linear RGB.
    pub color: Option<[f32; 3]>,
}

/// What a signal-bound TCP effect renders as.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FlashKind {
    /// Additive arc sprite + light at the TCP (spot welding).
    #[default]
    Flash,
    /// An accumulating cut trace along the TCP, plus the optional
    /// spinning link (machining).
    Trace,
    /// A translucent cone from the TCP along its -Z while the signal is
    /// on (spray painting). Bind it to the effective trigger
    /// (`SequenceTimeline::with_trigger_signal`) so it follows what
    /// actually sprayed, not the enable alone.
    Spray,
    /// The bound link spun about its own Z while the signal is on, and
    /// nothing drawn (a screwdriver's bit, a nutrunner's socket).
    Spin,
    /// An obstacle lit in its colour while the signal is on — an indicator
    /// lamp on an operator panel, a tower light. The one effect that is
    /// not at a robot's TCP: `robot` is empty.
    Lamp,
}

/// A spray cone's size, meters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SprayCone {
    pub length: f64,
    pub radius: f64,
}

/// A scripted auxiliary device (PLC output): commanded by
/// [`Action::Device`], it moves obstacles kinematically each scan tick.
#[derive(Debug, Clone)]
pub struct Device {
    pub name: String,
    pub kind: DeviceKind,
}

#[derive(Debug, Clone)]
pub enum DeviceKind {
    /// Advects any *unattached* obstacle whose origin lies inside the zone
    /// box by `velocity` while running. Its running state is recorded as an
    /// output-signal lane under the device name.
    ///
    /// A conveyed part stops short of a *stopper* ahead of it — an
    /// obstacle flagged [`crate::Obstacle::stopper`] whose bounds lie
    /// across the part's path — and short of a part ahead that stopped,
    /// so parts queue (accumulate) behind a raised blade, deterministically
    /// and from the front. Objects a device drives (a blade on its
    /// cylinder) never ride the belt.
    ///
    /// With `encoder` the belt's travel is recorded as a word lane
    /// `<name>.encoder` (metres run since t = 0, REAL) the commanding
    /// controller reads.
    Conveyor {
        zone_pose: Isometry3<f64>,
        zone_size: Vector3<f64>,
        velocity: Vector3<f64>,
        running: bool,
        encoder: bool,
    },
    /// Moves the listed obstacles along `axis` at `speed`, positioned by
    /// `MoveTo` commands within `range`; `DeviceDone` means in-position.
    /// `stops` are named positions along the axis (a door's `closed` and
    /// `open`): [`DeviceCommand::MoveToStop`] drives to one, and each is a
    /// read-only input lane `<axis>/<stop>` — ON while the axis stands at
    /// it, the limit switch a door interlock is written from. Between
    /// stops every lane is off (MTConnect's UNLATCHED).
    ///
    /// `coils` make the axis a solenoid-driven cylinder: each `(coil,
    /// stop)` is an output coil `<axis>/<coil>` that drives the axis to
    /// the named stop when energised. A stop no coil names is the
    /// spring-return position — the axis goes there when its one coil
    /// drops (single solenoid); with a coil per stop (double solenoid)
    /// a dropped coil holds. The controller then sees the cylinder as
    /// coils and limit switches, never as a position word.
    ///
    /// With `encoder` the actual position is recorded as a word lane
    /// `<name>.encoder` (metres along `axis`, REAL) the commanding
    /// controller reads as feedback.
    LinearAxis {
        objects: Vec<String>,
        axis: Unit<Vector3<f64>>,
        speed: f64,
        position: f64,
        range: (f64, f64),
        stops: Vec<(String, f64)>,
        coils: Vec<(String, String)>,
        encoder: bool,
    },
    /// Feeds parked objects onto the line, one every `interval` seconds
    /// while running. Its running state is recorded as an output-signal
    /// lane, like a conveyor's.
    ///
    /// The pool is finite on purpose. A baked timeline is a *fixed set of
    /// named object tracks* — nothing can be born mid-run and still have a
    /// track or a USD prim — so an endless line is a magazine of carriers
    /// plus a [`DeviceKind::Sink`] that returns them. Member `i` waits at
    /// `park ∘ translate(pitch * i)`, which spreads the magazine out
    /// instead of stacking it at one point.
    ///
    /// A member that does not start on its parking slot starts out on the
    /// line: that is how a belt is authored already-loaded.
    Source {
        pool: Vec<String>,
        park: Isometry3<f64>,
        pitch: Vector3<f64>,
        /// Where a released member enters the line.
        pose: Isometry3<f64>,
        /// Seconds between releases. `0` makes it an *indexing* feeder:
        /// one carrier per [`DeviceCommand::Start`], nothing on its own.
        /// A sequence that names the carrier each step takes needs that —
        /// on a timer, a carrier released while the cell is busy goes past
        /// unclaimed and every later step is off by one.
        interval: f64,
        running: bool,
    },
    /// Returns any unattached object that reaches its zone to the named
    /// source's magazine, freeing it to be fed again. The end of a line,
    /// or the return run of a belt.
    Sink {
        zone_pose: Isometry3<f64>,
        zone_size: Vector3<f64>,
        /// Source device the object goes back to.
        source: String,
    },
    /// A guided transport vehicle (AGV / AMR seen from the cell): drives
    /// station to station along an authored path — straight legs at
    /// `speed`, in-place pivot turns at `turn_speed` — carrying its body
    /// obstacles rigidly. Commanded with [`DeviceCommand::Goto`];
    /// `DeviceDone` means parked at the commanded station (in-position,
    /// like an axis). Its moving state is an output-signal lane.
    ///
    /// The path is the model of the tape on the floor: travel is authored,
    /// never planned. The arrival heading is the last leg's direction, so
    /// the approach waypoint before a station is what sets how the vehicle
    /// docks.
    Vehicle {
        path: VehiclePath,
        /// Display-only wheels; their rotation is derived after the collision-checked bake.
        wheels: Vec<crate::wheels::VehicleWheel>,
        /// Obstacles carried rigidly as the vehicle's body.
        body: Vec<String>,
        /// Cruise speed on straight legs (m/s).
        speed: f64,
        /// Pivot-turn rate (rad/s).
        turn_speed: f64,
        /// Station the vehicle starts parked at.
        start: String,
        /// The drive semantics — how the path becomes motion, and what z
        /// profile the machine can honour.
        drive: Drive,
        /// Load deck, as a box *in the vehicle frame*: any unattached
        /// obstacle whose origin lies inside it rides along, rotation
        /// included. It is the conveyor's zone rule moved onto a moving
        /// frame — so a part placed on the deck simply joins the load, with
        /// no load/unload action to author and nothing to keep in step.
        tray: Option<(Isometry3<f64>, Vector3<f64>)>,
        /// Non-empty makes this a *formation* (design-cooperative-
        /// transport.md): several vehicles carrying one load as one
        /// machine. The path is the formation's — its frame is the load's
        /// — and each member's motion is derived from its slot. A goto
        /// boards the members (each parked on its slot, or the dispatch
        /// refuses by name) and carries them whole, the way a lift carries
        /// the vehicles in its car; on arrival they are free again.
        members: Vec<FormationMember>,
    },
    /// A lift (elevator): a car of obstacles moved along `axis` between
    /// named stops, carrying whatever its capture zone holds — loose parts
    /// by origin, and *vehicles* whole, their body, deck load and mounted
    /// robot riding the same rigid motion. Commanded with
    /// [`DeviceCommand::MoveToStop`]; `DeviceDone` is in-position at the
    /// stop. The cargo is fixed when the ride is commanded — an elevator
    /// moves after the doors close — and a vehicle half out of the zone
    /// refuses to board by name. Doors are ordinary authoring (a
    /// `LinearAxis` panel and a signal), not part of the device.
    Lift {
        /// Obstacles forming the car (floor plate, walls) — they ride too.
        car: Vec<String>,
        /// Capture zone at the reference position (`position = 0`), in the
        /// world frame; it rides `axis · position` with the car.
        zone_pose: Isometry3<f64>,
        zone_size: Vector3<f64>,
        /// Travel direction; +Z for the ordinary elevator.
        axis: nalgebra::Unit<Vector3<f64>>,
        speed: f64,
        /// Named stop positions along `axis`, metres from the reference.
        stops: Vec<(String, f64)>,
        /// The stop the car starts at.
        start: String,
    },
}

/// How a vehicle turns its path into motion — the drive semantics, and the
/// z profile the machine can honour. Every existing vehicle is the
/// differential drive; holonomic and aerial drives are later variants of
/// the same slot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Drive {
    /// Differential drive: face the leg, then drive it — straight legs at
    /// cruise speed, in-place pivot turns between them.
    Differential {
        /// May it drive a leg backwards rather than turn around for it?
        /// A differential-drive machine backs out of a dead end instead of
        /// pirouetting in it — and a pivot sweeps the body's half-diagonal,
        /// so in a dock that is often the difference between fitting and
        /// not. Off (the default) keeps every arrival nose-first.
        allow_reverse: bool,
        /// Steepest grade the machine may climb, as rise over horizontal
        /// run (0.10 = 10 %). `None` means level paths only: a path that
        /// climbs is refused by name until the machine declares what it
        /// can do.
        max_grade: Option<f64>,
        /// Cruise speed of a leg driven in reverse (m/s); `None` backs as
        /// fast as it drives forward. An automated forklift travels
        /// drive-unit first at several times its forks-first speed, so the
        /// gear a leg is taken in prices it — and, stated, decides it: a
        /// corner then takes the gear that has the leg done soonest, turn
        /// and run together (a time tie goes to the shorter turn, a full
        /// tie to `prefer`). Unstated, the shorter turn wins.
        reverse_speed: Option<f64>,
        /// The gear a corner takes when facing the next leg costs the same
        /// turn either way — a right angle. `Forward` is the historic rule;
        /// a machine that travels drive-unit first says `Reverse` and then
        /// backs down the aisle it just turned into. Read only with
        /// `allow_reverse`; a station's arrival gear overrides it on the
        /// leg into that station.
        prefer: Gear,
    },
    /// A steered drive that cannot turn on the spot — one steered wheel
    /// against a fixed axle steered short of across, or an Ackermann axle:
    /// a counterbalance forklift. It rounds every corner with an arc of
    /// `turn_radius` about a centre beside its fixed axle's midpoint (the
    /// vehicle frame), and where the gear changes at a corner it drives
    /// past the corner and swings back through the arc — the operator's
    /// overshoot and reverse. Level paths only; a station is entered and
    /// left along one line (the corner goes on the waypoint before it),
    /// and a leg must be long enough for the turns at its ends. Gears are
    /// chosen as a differential drive chooses them.
    Steered {
        /// The vehicle frame's turning radius, metres (the fixed axle's
        /// midpoint, not the type sheet's outer radius Wa — that is what
        /// the body sweeps about it).
        turn_radius: f64,
        allow_reverse: bool,
        reverse_speed: Option<f64>,
        prefer: Gear,
    },
    /// Holonomic drive (mecanum / omni wheels): the machine translates in
    /// any direction while holding its heading — no pivot turns, ever.
    /// It docks facing whatever it faced when parked, which is the whole
    /// point of buying those wheels. The z rules are a ground drive's
    /// (grade within `max_grade`, vertical stacks are a lift's job).
    Holonomic {
        /// Steepest grade the machine may climb (see `Differential`).
        max_grade: Option<f64>,
    },
    /// Aerial drive (a multirotor): z is the machine's own axis, so the
    /// path may climb, dive or hang vertical legs with no grade rule and
    /// no lift. Each leg flies every axis at its own limit — the slower
    /// axis sets the clock, `T = max(run / speed, rise / climb (or
    /// descent))`, closed form. There is no takeoff command: a ground
    /// station next to an overhead waypoint *is* the takeoff, as a
    /// vertical leg.
    Aerial {
        /// Climb rate, m/s (positive).
        climb_speed: f64,
        /// Descent rate, m/s (positive).
        descent_speed: f64,
        /// What the nose does about +Z while flying.
        yaw: AerialYaw,
    },
}

/// An aerial drive's yaw policy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AerialYaw {
    /// Face each leg's horizontal course (a vertical leg keeps the
    /// heading it has).
    Course,
    /// Hold this yaw over the whole flight — a camera that must keep
    /// facing the racks.
    Fixed(f64),
}

/// Which way a differential-drive machine faces while it travels a leg:
/// `Forward` drives where it faces, `Reverse` backs along the leg (heading
/// = travel + π). A forklift's forks are its +X, so `Forward` is forks
/// first — how it enters a pallet — and `Reverse` is drive-unit first, how
/// it travels the aisle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gear {
    #[default]
    Forward,
    Reverse,
}

impl Drive {
    pub fn allow_reverse(&self) -> bool {
        match self {
            Drive::Differential { allow_reverse, .. } | Drive::Steered { allow_reverse, .. } => {
                *allow_reverse
            }
            Drive::Holonomic { .. } | Drive::Aerial { .. } => false,
        }
    }

    /// Does the drive face a leg — forward or in reverse? Holonomic and
    /// aerial drives have no gears to choose.
    pub fn has_gears(&self) -> bool {
        matches!(self, Drive::Differential { .. } | Drive::Steered { .. })
    }

    /// The turning radius of a steered drive; `None` pivots on the spot.
    pub fn turn_radius(&self) -> Option<f64> {
        match self {
            Drive::Steered { turn_radius, .. } => Some(*turn_radius),
            _ => None,
        }
    }

    pub fn max_grade(&self) -> Option<f64> {
        match self {
            Drive::Differential { max_grade, .. } | Drive::Holonomic { max_grade } => *max_grade,
            Drive::Steered { .. } | Drive::Aerial { .. } => None,
        }
    }

    /// The reverse cruise speed the drive states, if any. Stated, it also
    /// decides gears: a corner takes the gear that has the leg done soonest,
    /// turn and run together; unstated, the shorter turn.
    pub fn reverse_speed_stated(&self) -> Option<f64> {
        match self {
            Drive::Differential { reverse_speed, .. } | Drive::Steered { reverse_speed, .. } => {
                *reverse_speed
            }
            _ => None,
        }
    }

    /// The cruise speed of a leg driven in reverse, given the forward
    /// `speed`: the drive's own figure, or `speed` when it states none.
    pub fn reverse_speed(&self, speed: f64) -> f64 {
        self.reverse_speed_stated().unwrap_or(speed)
    }

    /// The gear a corner with no shorter side takes.
    pub fn prefer(&self) -> Gear {
        match self {
            Drive::Differential { prefer, .. } | Drive::Steered { prefer, .. } => *prefer,
            _ => Gear::Forward,
        }
    }
}

impl Default for Drive {
    fn default() -> Self {
        Drive::Differential {
            allow_reverse: false,
            max_grade: None,
            reverse_speed: None,
            prefer: Gear::Forward,
        }
    }
}

/// A named initial-state delta — one row of the cell's test-case matrix
/// (the FAT scenario list, moved ahead of the build). Deltas only, so
/// the cell stays single-source: one program, different worlds. The
/// unmodified scene is the reserved scenario `baseline`.
///
/// Determinism is untouched: a scenario is applied to a snapshot before
/// the rollout, and each (scenario, sequence set) pair bakes the same
/// timeline every time — which is what makes branch coverage and
/// per-scenario cycle times CI-assertable numbers.
#[derive(Debug, Clone)]
pub struct Scenario {
    pub name: String,
    /// Internal-signal initial values to override (declared signals only
    /// — sensors are geometric and follow from the world).
    pub signals: Vec<(String, bool)>,
    /// Obstacle poses to override. Attached obstacles are refused:
    /// moving one re-grasps it, which is a live-editing gesture, not a
    /// world variation.
    pub obstacles: Vec<(String, Isometry3<f64>)>,
    /// `(robot instance, joint positions)` start configurations.
    pub joints: Vec<(String, Vec<f64>)>,
    /// Inputs forced for the whole run — the fourth delta. Where
    /// `signals` sets an initial value the programs then overwrite, a
    /// fault pins a sensor or an internal signal to one value from t = 0
    /// on: the sensor's geometry is ignored, a program's `set` on the
    /// signal is dropped. The wire that broke, moved into the test matrix.
    pub faults: Vec<Fault>,
}

/// One forced input of a scenario: a stuck contact or a broken wire.
#[derive(Debug, Clone, PartialEq)]
pub struct Fault {
    /// A sensor name or an internal-signal name — something with an input
    /// lane. Device run lanes are outputs and cannot be forced; the DIs a
    /// `device_done` / `robot_done` wait reads have no lane of their own.
    pub target: String,
    pub kind: FaultKind,
}

/// How an input is forced. Both hold from the first scan to the end of the
/// run; injection at a step or an edge is a later, anchored form — a
/// scenario never carries an absolute time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaultKind {
    /// The contact is stuck at this value.
    StuckAt(bool),
    /// The wire is open: the input level is low, so the *functional*
    /// value is whatever the point's binding makes of a low level —
    /// `false` on a normally-open wiring, `true` on an inverted one, and
    /// `false` when nothing is bound. Whether the cell fails safe under a
    /// broken wire is exactly what this shows. Only inputs can be opened.
    Open,
    /// The target is an I/O node (a controller or a station) that dropped
    /// off: every input wired on it — and on the stations uplinked to it
    /// — reads as an open wire. The communication loss of the I/O map,
    /// resolved through the bindings at simulate time.
    NodeDown,
}

impl FaultKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            FaultKind::StuckAt(_) => "stuck",
            FaultKind::Open => "open",
            FaultKind::NodeDown => "node_down",
        }
    }
}

/// The reserved name for the unmodified scene.
pub const BASELINE_SCENARIO: &str = "baseline";

/// An authored guide path. Waypoints are points on the guidance surface —
/// z is the floor height there, so a flat cell authors z = 0 and a ramp
/// climbs with its waypoints. Stations are the named stops a `Goto` can
/// target, as waypoint indices — the point-table mental model of a PLC
/// positioning unit.
#[derive(Debug, Clone)]
pub struct VehiclePath {
    pub waypoints: Vec<nalgebra::Point3<f64>>,
    /// `(name, waypoint index)` pairs.
    pub stations: Vec<(String, usize)>,
    /// A closed loop: a goto walks whichever way around is shorter.
    pub ring: bool,
    /// `(station, gear)` pairs: the gear the leg *into* that station is
    /// driven in, whatever the corner rule would have picked — forks
    /// first into a pallet, drive-unit first onto a charger. A station
    /// not listed arrives by the rule. Differential drives only.
    pub arrivals: Vec<(String, Gear)>,
}

impl VehiclePath {
    /// Waypoint index of the named station.
    pub fn station(&self, name: &str) -> Option<usize> {
        self.stations
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, i)| *i)
    }

    /// The gear a goto must arrive at the named station in, if stated.
    pub fn arrival(&self, station: &str) -> Option<Gear> {
        self.arrivals
            .iter()
            .find(|(n, _)| n == station)
            .map(|(_, g)| *g)
    }

    /// The heading a vehicle parked at waypoint `at` faces: along the leg
    /// leaving it (wrapping on a ring), or along the leg arriving when
    /// nothing leaves — the open path's last waypoint.
    pub fn heading_at(&self, at: usize) -> f64 {
        let n = self.waypoints.len();
        let dir = |i: usize, j: usize| {
            let d = self.waypoints[j] - self.waypoints[i];
            // Heading is about +Z, so only the horizontal run can set it.
            (d.x.hypot(d.y) > 1e-9).then(|| d.y.atan2(d.x))
        };
        for step in 1..n {
            let j = if self.ring {
                (at + step) % n
            } else if at + step < n {
                at + step
            } else {
                break;
            };
            if let Some(h) = dir(at, j) {
                return h;
            }
        }
        for step in 1..n {
            let j = if self.ring {
                (at + n - (step % n)) % n
            } else if step <= at {
                at - step
            } else {
                break;
            };
            if let Some(h) = dir(j, at) {
                return h;
            }
        }
        0.0
    }

    /// The SE(2) frame of a vehicle parked at the named station.
    pub fn frame_at(&self, station: &str) -> Option<Isometry3<f64>> {
        let at = self.station(station)?;
        let p = self.waypoints.get(at)?;
        Some(vehicle_frame(p, self.heading_at(at)))
    }
}

/// The tape a steered drive follows, as a polyline for drawing: every
/// waypoint that is not a station is rounded with the fillet arc of
/// `radius` tangent to both legs — what a same-gear pass drives — when the
/// legs beside it have room for it; a corner with no room, and every
/// station, stays sharp. The studio keeps the same mirror in TS.
pub fn filleted_path(path: &VehiclePath, radius: f64) -> Vec<[f64; 2]> {
    let n = path.waypoints.len();
    let mut out = Vec::with_capacity(n * 4);
    for i in 0..n {
        let p = path.waypoints[i];
        let station = path.stations.iter().any(|(_, s)| *s == i);
        let (prev, next) = if path.ring && n > 2 {
            (Some((i + n - 1) % n), Some((i + 1) % n))
        } else {
            (i.checked_sub(1), (i + 1 < n).then_some(i + 1))
        };
        let (Some(a), Some(b)) = (prev, next) else {
            out.push([p.x, p.y]);
            continue;
        };
        let (a, b) = (path.waypoints[a], path.waypoints[b]);
        let (u, v) = (p - a, b - p);
        let (la, lb) = (u.x.hypot(u.y), v.x.hypot(v.y));
        if station || la < 1e-9 || lb < 1e-9 {
            out.push([p.x, p.y]);
            continue;
        }
        let (ux, uy, vx, vy) = (u.x / la, u.y / la, v.x / lb, v.y / lb);
        let delta = (ux * vy - uy * vx).atan2(ux * vx + uy * vy);
        let tangent = radius * (delta.abs() / 2.0).tan();
        if delta.abs() < 1e-6 || tangent > la / 2.0 || tangent > lb / 2.0 {
            out.push([p.x, p.y]);
            continue;
        }
        let (sx, sy) = (p.x - ux * tangent, p.y - uy * tangent);
        let (cx, cy) = (
            sx - uy * radius * delta.signum(),
            sy + ux * radius * delta.signum(),
        );
        let a0 = (sy - cy).atan2(sx - cx);
        for j in 0..=8 {
            let ang = a0 + delta * (j as f64) / 8.0;
            out.push([cx + radius * ang.cos(), cy + radius * ang.sin()]);
        }
    }
    out
}

/// The frame of a vehicle: its position on the guidance surface (z is the
/// floor height there) plus heading about +Z. The body stays level — pitch
/// and roll never enter, on a ramp the machine translates up it.
pub fn vehicle_frame(position: &nalgebra::Point3<f64>, heading: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        nalgebra::Translation3::new(position.x, position.y, position.z),
        nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), heading),
    )
}

/// One vehicle of a formation and where it stands under the load.
#[derive(Debug, Clone, PartialEq)]
pub struct FormationMember {
    /// The member vehicle's device name.
    pub vehicle: String,
    /// The member's vehicle frame in the formation's frame — its slot.
    pub slot: Isometry3<f64>,
}

/// A robot riding on a vehicle: its base is `vehicle frame ∘ offset`,
/// re-derived every scan tick. Mounting is what makes an AMR out of an arm
/// and a chassis, and it is deliberately an attribute of the *robot* — the
/// vehicle knows nothing about its passenger.
#[derive(Debug, Clone)]
pub struct RobotMount {
    /// Vehicle device name.
    pub device: String,
    /// Where the robot's base sits in the vehicle's frame.
    pub offset: Isometry3<f64>,
    /// Catalog frame snapshot used to review this mount, not its visual body.
    pub reference: Option<crate::mounting::VehicleMountReference>,
    /// Set when the robot *is* the vehicle's legs: while the vehicle
    /// drives, the gait swings these legs so that every planted foot stays
    /// where it touched down. Not an action — a property of the mount, the
    /// way a wheel's spin is a property of the axle it sits on.
    pub gait: Option<GaitSpec>,
    /// Presentation only: joints spun at a constant signed rate (rad/s)
    /// while the vehicle is off its starting ground or moving at all — a
    /// multirotor's propellers. No verdict reads the phase: the checking
    /// shape stays the swept solid the catalog authors (design-drone.md
    /// §3.4); this is the picture no longer lying about a parked prop.
    /// Continuous joints only — a limited joint would be driven through
    /// its stops.
    pub spin: Vec<(String, f64)>,
    /// Set when the robot *is* the vehicle's wheels: a humanoid upper body
    /// on a wheeled base, modelled whole. `Some` is itself the statement
    /// "this machine rolls on the floor its vehicle drives over" — the
    /// machine is one purchase, and a walkable surface under it is the
    /// arrangement, not a collision — so a model whose wheels are welded
    /// into the chassis mesh declares it with no wheels to turn.
    ///
    /// Set *with* `gait` for a wheel-legged machine — wheels for feet, one
    /// hanging from each foot link (design-wheel-legged.md): the legs are
    /// the gait's, the wheel joints the drive's, and
    /// [`WheelDrive::mode`] says which of the two takes the route.
    pub drive: Option<WheelDrive>,
}

/// How a mounted robot rolls. The vehicle's checked motion is the only
/// input: each wheel joint turns by exactly what its hub travelled, the
/// way [`crate::wheels`] turns an obstacle wheel — no forces, no slip. The
/// joints are the mount's to drive (one driver per joint): no motion or
/// ramp may own them.
#[derive(Debug, Clone, PartialEq)]
pub struct WheelDrive {
    pub wheels: Vec<MountWheel>,
    /// The link whose origin is the machine's point on the floor — under
    /// the turn centre, the URDF's `base_footprint`. It is what rides the
    /// vehicle frame when the mount states no offset.
    pub base_frame: Option<String>,
    /// How a mount that has legs *and* wheels — a wheel-legged quadruped
    /// (design-wheel-legged.md) — takes its vehicle's route. Read only
    /// when the mount also carries a gait: a machine that only rolls
    /// rolls.
    pub mode: LocomotionMode,
    /// The tallest break in the floor the wheels roll over, metres — a
    /// kerb, a threshold. Along a leg of the route, the supporting surface
    /// is read every wheel radius; a jump beyond this between two readings
    /// is a step, and a step is walked (or, on a mount that only rolls,
    /// refused by name). Zero — the default, and the honest figure when
    /// none is published — walks every step.
    pub max_step: f64,
}

/// Which running gear a wheel-legged machine takes a route on. The wheels
/// hang from the feet, so the two cannot both be true of one instant: a
/// planted axle is a locked wheel, a rolling hub is a leg held still.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LocomotionMode {
    /// Decided leg by leg of the route from the floor under it: rolled
    /// where the floor is continuous, walked where it steps (a stair
    /// flight), a turn walked when either straight beside it is, a roll
    /// too short to be worth the change walked with its neighbours.
    #[default]
    Auto,
    /// The legs hold the posture the machine was mounted in and every
    /// wheel turns by what its hub travelled — a skid-steered cart with
    /// legs for suspension. A step on the route is refused by name.
    Roll,
    /// The wheels are locked and the machine walks on them as on feet:
    /// the legged machine it also is, wheel radius for foot radius.
    Walk,
}

impl LocomotionMode {
    /// The name the Python API and the project file spell it by.
    pub fn as_str(self) -> &'static str {
        match self {
            LocomotionMode::Auto => "auto",
            LocomotionMode::Roll => "roll",
            LocomotionMode::Walk => "walk",
        }
    }

    /// The inverse of [`LocomotionMode::as_str`].
    pub fn parse(name: &str) -> Result<Self, String> {
        match name {
            "auto" => Ok(LocomotionMode::Auto),
            "roll" => Ok(LocomotionMode::Roll),
            "walk" => Ok(LocomotionMode::Walk),
            other => Err(format!(
                "mode must be \"auto\", \"roll\" or \"walk\", got {other:?}"
            )),
        }
    }
}

/// One wheel of a [`WheelDrive`]. The axle and the hub are the joint's own
/// axis and origin; only what the model cannot say is stated.
#[derive(Debug, Clone, PartialEq)]
pub struct MountWheel {
    /// The wheel's continuous joint.
    pub joint: String,
    /// Effective rolling radius in metres.
    pub radius: f64,
    /// Lateral travel / rolling travel coupling, as on
    /// [`crate::wheels::VehicleWheel`]: 0 for an ordinary wheel, -1 / +1
    /// for the two diagonals of a 45-degree mecanum arrangement.
    pub lateral_ratio: f64,
    /// A swerve module's steering joint (about the vertical): aimed along
    /// the hub's travel before the wheel rolls.
    pub steer: Option<String>,
}

/// How a mounted robot walks. Authored once per machine (a catalog package
/// declares it); the scan engine derives everything else from the vehicle's
/// motion. See design/design-legged.md §4.
#[derive(Debug, Clone, PartialEq)]
pub struct GaitSpec {
    /// The link the legs hang from; `None` is the root link.
    pub body_link: Option<String>,
    /// The legs, in the order the pattern's phase table is read.
    pub legs: Vec<LegSpec>,
    pub pattern: GaitPattern,
    /// Cycle period in seconds.
    pub period: f64,
    /// Swing apex above the ground, metres.
    pub lift: f64,
    /// The standing configuration as `(joint, value)` pairs. Must name
    /// every leg joint; other joints keep their mount-time value.
    pub stance: Vec<(String, f64)>,
    /// Longest stride the legs can take between two landings, metres.
    /// `speed · period` (and the pivot's outer-foot arc) must stay under it.
    pub max_stride: f64,
    /// How far the foot link's origin stands above the floor: a ball
    /// foot's radius, or an ankle frame's height over its sole. Zero for a
    /// frame on the sole itself.
    pub foot_radius: f64,
    /// Joints swung in time with the first leg, `(joint, amplitude)` —
    /// a biped's arms. Not swung while the robot holds something or a
    /// ramp is driving them.
    pub arm_swing: Vec<(String, f64)>,
    /// Body sway while walking, metres: `bob` up and down twice a cycle,
    /// `lateral` over the planted leg once a cycle. The legs absorb it; the
    /// feet do not move. Zero is a body that rides rigidly, as a quadruped
    /// at a trot very nearly does.
    pub bob: f64,
    pub lateral: f64,
    /// Tallest step (rise between two consecutive footholds of one leg,
    /// metres) the machine may take. `None` skips the declared check and
    /// leaves unreachable steps to the IK (`GaitReach`); a catalog package
    /// fills it from `max_step_height_mm`. Also sets how far above/below
    /// the vehicle plane a walkable surface is searched for a foothold.
    pub max_step: Option<f64>,
    /// The pace the machine walks at, m/s, on the legs of a route it
    /// walks while its vehicle would roll faster — a wheel-legged machine's
    /// stair flights (design-wheel-legged.md §3.5). `None` takes the
    /// stride-safe `0.6 · max_stride / period`. A machine that only walks
    /// walks at its vehicle's speed and does not read this.
    pub speed: Option<f64>,
    /// Likewise the pivot rate it walks turns at, rad/s; `None` keeps the
    /// vehicle's.
    pub turn_speed: Option<f64>,
    /// How much of a tread a foot needs around its contact point, metres —
    /// the margin a foothold keeps from a tread's edge, and the disc a
    /// foothold is fitted on. `None` takes `foot_radius`, a ball foot's
    /// own; a wheel touches at a point and needs only a contact patch
    /// (20 mm), not its radius of tread.
    pub foothold: Option<f64>,
}

impl GaitSpec {
    /// The rates a wheel-legged machine walks the walked legs of a route
    /// at, given its vehicle's rolling rates: the declared pace, else the
    /// stride-safe `0.6 · max_stride / period`, and never faster than the
    /// vehicle; the declared pivot rate, else the vehicle's.
    pub fn walk_rates(&self, speed: f64, turn_speed: f64) -> (f64, f64) {
        let pace = self.speed.unwrap_or(0.6 * self.max_stride / self.period);
        (
            speed.min(pace),
            turn_speed.min(self.turn_speed.unwrap_or(turn_speed)),
        )
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LegSpec {
    /// Display name (`FL`, `L`, ...).
    pub name: String,
    /// The foot link: its origin is what touches the floor.
    pub foot: String,
    pub contact: FootContact,
}

/// What a foot is to the floor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FootContact {
    /// A ball or a point: position only, any orientation.
    Point,
    /// A sole that lies flat on the floor; `yaw_free` lets the foot point
    /// where the leg's DOF take it.
    Sole { yaw_free: bool },
}

/// Which legs fly when. The built-ins are laid over the legs in declaration
/// order — `FL, FR, RL, RR` for the quadruped patterns, `L, R` for biped.
#[derive(Debug, Clone, PartialEq)]
pub enum GaitPattern {
    /// Lateral-sequence walk, duty 0.75: three feet down at all times.
    Walk,
    /// Diagonal pairs in antiphase, duty 0.5 — the quadruped's cruise.
    Trot,
    /// Alternating legs, duty 0.6.
    Biped,
    /// Any duty factor and per-leg phases in `[0, 1)`.
    Custom { duty: f64, phases: Vec<f64> },
}

#[derive(Debug, Clone)]
pub enum DeviceCommand {
    Start,
    Stop,
    SetSpeed(f64),
    MoveTo(f64),
    /// Send a vehicle to a named station (the dispatch order); await it
    /// with `DeviceDone`.
    Goto {
        station: String,
    },
    /// Indexed transfer: run a *stopped* conveyor for exactly `distance`
    /// metres along its velocity direction, then stop; await it with
    /// `DeviceDone`. The scan loop consumes one `v·dt` per tick and the
    /// final tick moves exactly the remainder, so the pitch is exact no
    /// matter how the scan period divides it — this is what retires the
    /// `elapsed(pitch/v + one tick)` arithmetic an indexing line otherwise
    /// needs (and the silent 1-scan shortfall when it is forgotten).
    Advance(f64),
    /// Send a lift to a named stop; await it with `DeviceDone`. The
    /// cargo — vehicles and loose parts in the capture zone — is fixed
    /// the moment this fires.
    MoveToStop(String),
}

#[derive(Debug, Clone)]
pub struct Sequence {
    pub name: String,
    pub steps: Vec<Step>,
}

#[derive(Debug, Clone)]
pub struct Step {
    /// Display name (「搬入待ち」…); shown on the timeline band.
    pub name: String,
    /// Entry actions, fired once when the step becomes active.
    pub actions: Vec<Action>,
    /// The step completes (and the next begins) when this holds.
    pub transition: Condition,
    /// SFC selection divergence: when non-empty, this step is a branching
    /// step — it carries no actions, its `transition` is unused
    /// (`Immediately` by convention), and the arms' conditions are its
    /// outgoing transitions. The first arm whose condition holds wins
    /// (authored order = priority, SFC's left-to-right rule); its steps
    /// run and every arm rejoins at the step after this one. Structured
    /// on purpose: no jump targets means no backward edges, so a rolled
    /// timeline always terminates and stays a single deterministic path.
    pub select: Vec<SelectArm>,
}

/// One arm of a selection divergence: an outgoing transition plus the
/// steps it guards. Arms may nest further branches; an empty arm is a
/// skip straight to the rejoin ("already done → carry on").
#[derive(Debug, Clone)]
pub struct SelectArm {
    pub condition: Condition,
    pub steps: Vec<Step>,
}

/// Visits every action in `steps`, branch arms included, in authoring
/// order.
pub(crate) fn walk_actions<E>(
    steps: &[Step],
    f: &mut impl FnMut(&Action) -> Result<(), E>,
) -> Result<(), E> {
    for step in steps {
        for action in &step.actions {
            f(action)?;
        }
        for arm in &step.select {
            walk_actions(&arm.steps, f)?;
        }
    }
    Ok(())
}

/// Every branching step in `steps`, pre-order (a select first, then its
/// arms' steps in arm order) — a select's index in this list is its
/// ordinal, the number `BranchTaken.select` carries. The rollout's
/// flattening assigns ordinals by the same walk; a test pins the two
/// against each other.
pub(crate) fn enumerate_selects(steps: &[Step]) -> Vec<&Step> {
    fn walk<'a>(steps: &'a [Step], out: &mut Vec<&'a Step>) {
        for step in steps {
            if !step.select.is_empty() {
                out.push(step);
            }
            for arm in &step.select {
                walk(&arm.steps, out);
            }
        }
    }
    let mut out = Vec::new();
    walk(steps, &mut out);
    out
}

/// Robot-addressed actions carry `robot: Option<String>` — the instance
/// name, or `None` for the scene's sole robot. With several robots `None`
/// is ambiguous and rejected at validation (same rule as the Python API).
#[derive(Debug, Clone)]
pub enum Action {
    /// Start a named motion (planned against the world state at this
    /// moment; the motion's owning robot drives). Await it with
    /// [`Condition::Done`].
    StartMotion { motion: String },
    /// Linearly ramp a subset of joints (a gripper open/close) over
    /// `duration` seconds. Await it with [`Condition::Done`]. A ramp is not
    /// planned; with `check`, its straight joint line is checked against
    /// the world as it stands when the ramp starts — as a planned motion's
    /// is — and a contact fails the bake (an arm moved by ramps; a gripper
    /// closing on its part is the usual unchecked ramp).
    StartRamp {
        robot: Option<String>,
        targets: Vec<(String, f64)>,
        duration: f64,
        check: bool,
    },
    /// Hand the robot — or one arm (`group`) — to a registered policy for
    /// this step: a learned or scripted controller supplied at bake time
    /// (`simulate_sequence(policies=...)`), asked for a joint target every
    /// `1/hz` seconds until it declares itself done or `max_duration`
    /// runs out. The joints move toward each target under the drive's
    /// rate limit (the model's velocity limits), so a policy steers but
    /// never teleports the arm; a collision with the scenery while it
    /// drives fails the bake. Await it with [`Condition::Done`]
    /// (design-rl.md R3).
    Policy {
        policy: String,
        robot: Option<String>,
        group: Option<String>,
        hz: f64,
        max_duration: f64,
    },
    /// Grasp: rigidly attach an obstacle at its current relative pose
    /// (defaults as in [`Scene::attach_obstacle`]). Instantaneous.
    Attach {
        robot: Option<String>,
        object: String,
        link: Option<String>,
        touch_links: Option<Vec<String>>,
        /// The arm that grasps (a dual-arm robot): `link` then defaults to
        /// its tip. `None` on a robot with one group.
        group: Option<String>,
    },
    /// Conveyor tracking: latch onto a moving part. Until the track is
    /// released, every commanded pose is carried by the part's motion since
    /// this instant — poses taught at the station keep meeting the part
    /// while it travels, so the line never has to stop. Grasping the tracked
    /// part freezes the offset (it is the robot's now); the offset survives
    /// [`Action::Untrack`], which only stops following. Instantaneous.
    Track {
        robot: Option<String>,
        object: String,
        /// Link servoed onto the part; defaults to the TCP link.
        link: Option<String>,
        /// The arm that follows (a dual-arm robot): only its joints are
        /// spent on the track, so the other arm may drive on — or carry
        /// the very part this arm follows (a two-handed hold is one hand
        /// grasping and the other tracking).
        group: Option<String>,
    },
    /// Release the track: commands go back to plain world coordinates from
    /// wherever the robot stands (never a jump). Instantaneous.
    Untrack {
        robot: Option<String>,
        /// The arm whose track to release (a robot tracking with both).
        group: Option<String>,
    },
    /// Release: the obstacle's pose freezes where it is. Instantaneous.
    /// The carrying robot is looked up from the attachment.
    Detach { object: String },
    /// Write an internal signal (self-holding relay style).
    Set { signal: String, value: bool },
    /// Write a word (a transfer, or one arithmetic instruction): a
    /// setpoint moved in on a step's entry, a rung's `MOV`.
    SetWord { word: String, value: WordExpr },
    /// Command an auxiliary device (output coil: conveyor start/stop,
    /// axis positioning). Instantaneous state change.
    Device {
        device: String,
        command: DeviceCommand,
    },
    /// Start a toolpath (a continuous Cartesian process path — milling,
    /// trimming) on `robot`: baked against the world at this moment as an
    /// automatic joint-space approach to the path start followed by the
    /// feed-floored follow. Await it with [`Condition::Done`].
    StartToolpath {
        robot: Option<String>,
        toolpath: String,
    },
    /// Lower a parked walker's body while it stands: over `duration` the
    /// body (its root link) goes from the crouch it is in to `depth`
    /// metres below its standing height, pitched nose-down by `lean`
    /// radians about its own lateral axis, on a smoothstep — every foot
    /// held where it stands and each leg re-solved every scan tick by the
    /// walk's own leg IK. `depth = 0, lean = 0` stands it back up.
    /// `None` takes the duration from a pace (0.3 m/s of depth, 0.6 rad/s
    /// of lean, peak 1.5 × mean, at least 0.6 s). Await it with
    /// [`Condition::Done`]; while the body is lowered its vehicle does not
    /// drive (a walk starts from the stance).
    Crouch {
        robot: Option<String>,
        depth: f64,
        lean: f64,
        duration: Option<f64>,
    },
}

#[derive(Debug, Clone)]
pub enum Condition {
    /// Always true: the step just fires its actions and moves on.
    Immediately,
    /// Every motion/ramp started by *this step* has finished (a step may
    /// start one per robot).
    Done,
    /// The named robot has no motion/ramp in flight — whichever step
    /// started it. The robot-level idle test interlocks are built from.
    RobotDone { robot: String },
    /// The named arm of the named robot has no motion/ramp in flight —
    /// nothing drives any of its joints. The per-arm idle test of a
    /// dual-arm robot.
    GroupDone { robot: String, group: String },
    /// On-delay timer: true `seconds` after the step became active.
    Elapsed { seconds: f64 },
    /// Level test of a signal (sensors join in S4 under the same name
    /// space).
    Signal { name: String, value: bool },
    /// Rising edge: the signal is on this scan and was off the previous
    /// one (PLC's `-|P|-`). True for exactly one scan per edge, so any
    /// number of programs see the same edge in that scan. No edge fires
    /// at t = 0 — startup state is not a transition.
    Rising { name: String },
    /// Falling edge: off this scan, on the previous one (`-|N|-`).
    Falling { name: String },
    /// A linear axis has reached its commanded position (in-position).
    DeviceDone { device: String },
    /// Series contacts (AND).
    All(Vec<Condition>),
    /// Parallel contacts (OR).
    Any(Vec<Condition>),
    /// The inverse of a condition (a normally-closed contact over a
    /// whole network).
    Not(Box<Condition>),
    /// A program stands in the named step — the step relay of a step
    /// ladder, read as a contact (output circuits are written from it).
    StepActive { sequence: String, step: String },
    /// A comparison contact over words: `dp >= dp_limit`, `lvl.raw < 0`.
    Compare {
        left: Operand,
        op: CompareOp,
        right: Operand,
    },
}

/// The continuous half of a program: rungs scanned every scan, after the
/// program's step transitions, top to bottom — and the gates of its step
/// chain (design-plc-resolution.md PF1). A program is a step chain
/// ([`Sequence`]), rungs, or both; stored beside the sequence of the same
/// name.
///
/// This is the evaluation of a rung model, not an IEC 61131-3 runtime:
/// contacts are [`Condition`] trees, outputs are the five of
/// [`RungOutput`].
#[derive(Debug, Clone, Default)]
pub struct Logic {
    pub name: String,
    pub rungs: Vec<Rung>,
    /// The step chain advances only while this holds (a step-mode switch,
    /// a pause): ANDed onto every transition. `None` = always.
    pub enable: Option<Condition>,
    /// While this holds the step chain is put back in — and held in — its
    /// first step (工程クリア). Moves already started run on.
    pub reset: Option<Condition>,
    /// The step chain starts over when it ends, in the same scan — the
    /// PLCopen export's jump to the first step, in the bake too.
    pub cycle: bool,
    /// The program is the equipment, not a controller: what a pump, a
    /// tank or a machine's own logic does (a pressure that climbs while
    /// liquid flows). It runs with the plant in every mode — batch, live,
    /// with the controllers outside — and is left out of the I/O map:
    /// what it reads and writes is physics, not wiring. A signal it reads
    /// is then an output of whoever writes it (the pump's contactor), one
    /// it writes an input of whoever reads it (a flow switch).
    pub plant: bool,
}

impl Logic {
    /// Nothing authored: no rungs and no gate.
    pub fn is_empty(&self) -> bool {
        self.rungs.is_empty()
            && self.enable.is_none()
            && self.reset.is_none()
            && !self.cycle
            && !self.plant
    }
}

/// One rung: a contact network and the outputs it drives.
#[derive(Debug, Clone)]
pub struct Rung {
    /// The rung's comment (shown over it; names it in tables).
    pub comment: String,
    pub condition: Condition,
    pub outputs: Vec<RungOutput>,
}

impl Rung {
    /// The name a rung goes by in tables: its comment, or `rung k`.
    pub fn label(&self, index: usize) -> String {
        if self.comment.trim().is_empty() {
            format!("rung {}", index + 1)
        } else {
            self.comment.clone()
        }
    }
}

/// What a rung drives.
#[derive(Debug, Clone, PartialEq)]
pub enum RungOutput {
    /// `-( )-`: the target follows the rung every scan. The target is an
    /// internal signal, a belt's or a feeder's run coil (the device's
    /// name), or a cylinder's solenoid coil (`<axis>/<coil>`).
    Coil { target: String },
    /// `-(S)-`: latches the signal on while the rung conducts.
    Set { signal: String },
    /// `-(R)-`: latches the signal off while the rung conducts; on a
    /// counter's signal it also clears the count.
    Reset { signal: String },
    /// On-delay timer: the signal comes on once the rung has conducted
    /// for `seconds`, and drops with it.
    Ton { signal: String, seconds: f64 },
    /// Up-counter: counts the rung's rising edges; the signal is on from
    /// `preset` counts until a `Reset` of it. The count is the word lane
    /// `<signal>.count`.
    Ctu { signal: String, preset: u32 },
    /// A transfer or an arithmetic instruction: while the rung conducts,
    /// the word is written with the expression every scan (`MOV`, `+`,
    /// `-`, `*`, `/`). Gate it with an edge contact to run it once.
    Move { word: String, value: WordExpr },
}

impl RungOutput {
    /// The signal or coil this output writes.
    pub fn target(&self) -> &str {
        match self {
            RungOutput::Coil { target } => target,
            RungOutput::Set { signal }
            | RungOutput::Reset { signal }
            | RungOutput::Ton { signal, .. }
            | RungOutput::Ctu { signal, .. } => signal,
            RungOutput::Move { word, .. } => word,
        }
    }
}

/// What a coil's target turned out to be.
#[derive(Debug, Clone, PartialEq)]
pub enum CoilTarget {
    /// An internal signal.
    Signal,
    /// A belt's or a feeder's run coil.
    Run,
    /// A solenoid coil of an axis: energised it drives to `stop`; dropped,
    /// a single-solenoid axis returns to `rest`.
    AxisCoil {
        device: String,
        stop: String,
        rest: Option<String>,
    },
}

/// One arm's track while a sequence is walked for validation: which arm
/// (`None` = the whole robot), the joints it spends, what it follows, and
/// the joints a ramp must not touch meanwhile.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct TrackClaim {
    pub(crate) group: Option<usize>,
    pub(crate) joints: Vec<usize>,
    pub(crate) object: String,
    pub(crate) contested: Vec<String>,
    /// The tool mount's name, for the ramp error's advice.
    pub(crate) mount: String,
}

impl Action {
    /// The instance name this action is addressed to, when it carries one.
    /// `StartMotion` is absent on purpose: it names a motion, and the
    /// motion knows its own robot.
    pub(crate) fn robot_mut(&mut self) -> Option<&mut String> {
        match self {
            Action::StartRamp { robot, .. }
            | Action::Policy { robot, .. }
            | Action::Attach { robot, .. }
            | Action::Track { robot, .. }
            | Action::Untrack { robot, .. }
            | Action::StartToolpath { robot, .. }
            | Action::Crouch { robot, .. } => robot.as_mut(),
            _ => None,
        }
    }
}

impl Condition {
    fn mentions_done(&self) -> bool {
        match self {
            Condition::Done => true,
            Condition::All(cs) | Condition::Any(cs) => cs.iter().any(|c| c.mentions_done()),
            Condition::Not(c) => c.mentions_done(),
            _ => false,
        }
    }

    /// Whether the condition is relative to a step (its moves, its entry
    /// time) — meaningless on a rung or a gate, which belong to no step.
    fn step_relative(&self) -> bool {
        match self {
            Condition::Done | Condition::Elapsed { .. } => true,
            Condition::All(cs) | Condition::Any(cs) => cs.iter().any(|c| c.step_relative()),
            Condition::Not(c) => c.step_relative(),
            _ => false,
        }
    }
}

// ------------------------------------------------------------- scene state

impl Scene {
    pub fn sequences(&self) -> &[Sequence] {
        &self.sequences
    }

    pub fn sequence(&self, name: &str) -> Option<&Sequence> {
        self.sequences.iter().find(|s| s.name == name)
    }

    /// Adds or replaces a sequence wholesale (steps are small; the studio
    /// edits locally and sends the full list, like motions).
    pub fn upsert_sequence(&mut self, sequence: Sequence) {
        match self.sequences.iter_mut().find(|s| s.name == sequence.name) {
            Some(slot) => *slot = sequence,
            None => self.sequences.push(sequence),
        }
    }

    pub fn remove_sequence(&mut self, name: &str) -> Result<(), SceneError> {
        let before = self.sequences.len();
        self.sequences.retain(|s| s.name != name);
        if self.sequences.len() == before {
            return Err(SceneError::UnknownSequence(name.to_string()));
        }
        self.logics.retain(|l| l.name != name);
        Ok(())
    }

    /// The rungs and gates of every program that has any.
    pub fn logics(&self) -> &[Logic] {
        &self.logics
    }

    /// The rungs and gates of program `name`, when it has any.
    pub fn logic(&self, name: &str) -> Option<&Logic> {
        self.logics.iter().find(|l| l.name == name)
    }

    /// Sets (or, when empty, clears) the rungs and gates of program
    /// `logic.name`. A program with rungs alone — no step chain — is a
    /// sequence with no steps; one is made when none exists.
    pub fn upsert_logic(&mut self, logic: Logic) {
        self.logics.retain(|l| l.name != logic.name);
        if logic.is_empty() {
            return;
        }
        // A timer's or a counter's signal is declared by its use (a PLC's
        // T / C device); coils and latches write relays the cell defines.
        for output in logic.rungs.iter().flat_map(|r| &r.outputs) {
            if let RungOutput::Ton { signal, .. } | RungOutput::Ctu { signal, .. } = output {
                if !self.signals.iter().any(|s| &s.name == signal) {
                    self.define_signal(signal, false);
                }
            }
        }
        if self.sequence(&logic.name).is_none() {
            self.sequences.push(Sequence {
                name: logic.name.clone(),
                steps: Vec::new(),
            });
        }
        self.logics.push(logic);
    }

    /// What a coil's target is: a belt's or feeder's run coil, an axis's
    /// solenoid coil, or an internal signal.
    pub fn coil_target(&self, target: &str) -> Result<CoilTarget, String> {
        if let Some(d) = self.devices.iter().find(|d| d.name == target) {
            return match d.kind {
                DeviceKind::Conveyor { .. } | DeviceKind::Source { .. } => Ok(CoilTarget::Run),
                _ => Err(format!(
                    "`{target}` has no run coil (a coil drives a conveyor, a source, a cylinder's                      `<axis>/<coil>`, or an internal signal)"
                )),
            };
        }
        if let Some((device, coil)) = target.rsplit_once('/') {
            if let Some(DeviceKind::LinearAxis { stops, coils, .. }) = self
                .devices
                .iter()
                .find(|d| d.name == device)
                .map(|d| &d.kind)
            {
                let stop = coils
                    .iter()
                    .find(|(c, _)| c == coil)
                    .map(|(_, s)| s.clone())
                    .ok_or_else(|| format!("axis `{device}` has no coil `{coil}`"))?;
                let rest = stops
                    .iter()
                    .map(|(s, _)| s)
                    .find(|s| !coils.iter().any(|(_, t)| &t == s))
                    .cloned();
                return Ok(CoilTarget::AxisCoil {
                    device: device.to_string(),
                    stop,
                    rest,
                });
            }
        }
        if self.signals.iter().any(|s| s.name == target) {
            return Ok(CoilTarget::Signal);
        }
        Err(format!(
            "unknown coil target `{target}` (define the signal, or name a conveyor, a source or              an axis coil `<axis>/<coil>`)"
        ))
    }

    /// A program's rungs as steps, for the walks that ask what a program
    /// reads and writes (the I/O map, the single-owner rule): each rung a
    /// step named by its label, its condition as the transition, its
    /// outputs as the actions they amount to. They follow the program's
    /// real steps, so rung `k` has flat index `flat steps + k`; the gates
    /// follow as steps `enable` and `reset`.
    pub fn rung_steps(&self, name: &str) -> Vec<Step> {
        let Some(logic) = self.logic(name) else {
            return Vec::new();
        };
        logic
            .rungs
            .iter()
            .enumerate()
            .map(|(k, rung)| Step {
                name: rung.label(k),
                actions: rung
                    .outputs
                    .iter()
                    .filter_map(|o| self.output_action(o, true))
                    .collect(),
                transition: rung.condition.clone(),
                select: Vec::new(),
            })
            // The gates read signals too: a step each, after the rungs.
            .chain(
                [("enable", &logic.enable), ("reset", &logic.reset)]
                    .into_iter()
                    .filter_map(|(name, gate)| {
                        gate.as_ref().map(|c| Step {
                            name: name.to_string(),
                            actions: Vec::new(),
                            transition: c.clone(),
                            select: Vec::new(),
                        })
                    }),
            )
            .collect()
    }

    /// The action an output fires when it turns `on` (or off): a write of
    /// its signal, a start / stop of its device, a move of its axis.
    /// `None` when turning it that way does nothing (a double-solenoid
    /// coil dropped) or the target does not resolve.
    pub fn output_action(&self, output: &RungOutput, on: bool) -> Option<Action> {
        let set = |signal: &String, value: bool| Action::Set {
            signal: signal.clone(),
            value,
        };
        match output {
            RungOutput::Set { signal } => Some(set(signal, true)),
            RungOutput::Reset { signal } => Some(set(signal, false)),
            RungOutput::Ton { signal, .. } | RungOutput::Ctu { signal, .. } => {
                Some(set(signal, on))
            }
            RungOutput::Move { word, value } => on.then(|| Action::SetWord {
                word: word.clone(),
                value: value.clone(),
            }),
            RungOutput::Coil { target } => match self.coil_target(target).ok()? {
                CoilTarget::Signal => Some(set(target, on)),
                CoilTarget::Run => Some(Action::Device {
                    device: target.clone(),
                    command: if on {
                        DeviceCommand::Start
                    } else {
                        DeviceCommand::Stop
                    },
                }),
                CoilTarget::AxisCoil { device, stop, rest } => {
                    let to = if on { Some(stop) } else { rest };
                    to.map(|stop| Action::Device {
                        device,
                        command: DeviceCommand::MoveToStop(stop),
                    })
                }
            },
        }
    }

    /// A program's step chain followed by its rungs as steps
    /// ([`rung_steps`](Self::rung_steps)) — what the usage walks read.
    pub fn program_steps(&self, sequence: &Sequence) -> Vec<Step> {
        let mut steps = sequence.steps.clone();
        steps.extend(self.rung_steps(&sequence.name));
        steps
    }

    /// Authoring-time validation of a program's rungs and gates.
    fn validate_logic(&self, logic: &Logic) -> Result<(), String> {
        let gate = |what: &str, c: &Option<Condition>| -> Result<(), String> {
            let Some(c) = c else { return Ok(()) };
            if c.step_relative() {
                return Err(format!(
                    "{what}: `done` / `elapsed` belong to a step — a gate reads signals"
                ));
            }
            self.validate_condition(c)
                .map_err(|m| format!("{what}: {m}"))
        };
        gate("enable", &logic.enable)?;
        gate("reset", &logic.reset)?;
        // One driver per signal: a coil, a timer or a counter owns its
        // target; SET / RST share theirs with each other (and a counter's
        // RST with its counter) but not with a coil.
        let mut driven: Vec<&str> = Vec::new();
        let mut latched: Vec<&str> = Vec::new();
        for (k, rung) in logic.rungs.iter().enumerate() {
            let at = format!("rung {} `{}`", k + 1, rung.label(k));
            if rung.condition.step_relative() {
                return Err(format!(
                    "{at}: `done` / `elapsed` belong to a step — time a rung with a `ton` output                      and read its signal"
                ));
            }
            self.validate_condition(&rung.condition)
                .map_err(|m| format!("{at}: {m}"))?;
            if rung.outputs.is_empty() {
                return Err(format!("{at}: a rung needs an output"));
            }
            for output in &rung.outputs {
                let target = output.target();
                match output {
                    RungOutput::Coil { target } => {
                        self.coil_target(target).map_err(|m| format!("{at}: {m}"))?;
                    }
                    RungOutput::Move { word, value } => {
                        self.validate_word_write(word, value)
                            .map_err(|m| format!("{at}: {m}"))?;
                    }
                    _ => {
                        if !self.signals.iter().any(|s| s.name == target) {
                            return Err(format!(
                                "{at}: unknown signal `{target}` (declare it with define_signal)"
                            ));
                        }
                    }
                }
                match output {
                    RungOutput::Ton { seconds, .. }
                        if !(seconds.is_finite() && *seconds >= 0.0) =>
                    {
                        return Err(format!("{at}: timer seconds must be >= 0, got {seconds}"));
                    }
                    RungOutput::Ctu { preset: 0, .. } => {
                        return Err(format!("{at}: a counter's preset is at least 1"));
                    }
                    _ => {}
                }
                match output {
                    RungOutput::Coil { .. } | RungOutput::Ton { .. } | RungOutput::Ctu { .. } => {
                        if driven.contains(&target) {
                            return Err(format!(
                                "{at}: `{target}` is driven by two outputs (a double coil) — one                                  coil, timer or counter per target"
                            ));
                        }
                        driven.push(target);
                    }
                    RungOutput::Set { .. } | RungOutput::Reset { .. } => latched.push(target),
                    // Several rungs may write one register, like SET / RST.
                    RungOutput::Move { .. } => {}
                }
            }
        }
        for target in latched {
            let counter = logic
                .rungs
                .iter()
                .flat_map(|r| &r.outputs)
                .any(|o| matches!(o, RungOutput::Ctu { signal, .. } if signal == target));
            if driven.contains(&target) && !counter {
                return Err(format!(
                    "`{target}` is both a coil / timer and set or reset — a latched signal is                      written by SET / RST alone"
                ));
            }
        }
        Ok(())
    }

    pub fn set_sequences(&mut self, sequences: Vec<Sequence>) {
        self.sequences = sequences;
    }

    pub fn signals(&self) -> &[SignalDef] {
        &self.signals
    }

    /// Declares (or re-initializes) an internal signal.
    pub fn define_signal(&mut self, name: &str, initial: bool) {
        match self.signals.iter_mut().find(|s| s.name == name) {
            Some(slot) => slot.initial = initial,
            None => self.signals.push(SignalDef {
                name: name.to_string(),
                initial,
            }),
        }
    }

    pub fn remove_signal(&mut self, name: &str) -> Result<(), SceneError> {
        let before = self.signals.len();
        self.signals.retain(|s| s.name != name);
        if self.signals.len() == before {
            return Err(SceneError::UnknownSignal(name.to_string()));
        }
        Ok(())
    }

    pub fn set_signals(&mut self, signals: Vec<SignalDef>) {
        self.signals = signals;
    }

    pub fn words(&self) -> &[WordDef] {
        &self.words
    }

    /// Declares (or re-initializes) an internal word.
    pub fn define_word(
        &mut self,
        name: &str,
        initial: f64,
        integer: bool,
    ) -> Result<(), SceneError> {
        if !initial.is_finite() {
            return Err(SceneError::BadWord(format!(
                "word `{name}`: the initial value must be finite, got {initial}"
            )));
        }
        if self.signal_readable(name) || self.analogs.iter().any(|a| a.name == name) {
            return Err(SceneError::BadWord(format!(
                "`{name}` is already a signal, a sensor or an analog input — a word needs its own name"
            )));
        }
        let initial = if integer { initial.round() } else { initial };
        match self.words.iter_mut().find(|w| w.name == name) {
            Some(slot) => {
                slot.initial = initial;
                slot.integer = integer;
            }
            None => self.words.push(WordDef {
                name: name.to_string(),
                initial,
                integer,
            }),
        }
        Ok(())
    }

    pub fn remove_word(&mut self, name: &str) -> Result<(), SceneError> {
        let before = self.words.len();
        self.words.retain(|w| w.name != name);
        if self.words.len() == before {
            return Err(SceneError::UnknownWord(name.to_string()));
        }
        Ok(())
    }

    pub fn set_words(&mut self, words: Vec<WordDef>) {
        self.words = words;
    }

    pub fn analogs(&self) -> &[Analog] {
        &self.analogs
    }

    pub fn analog(&self, name: &str) -> Option<&Analog> {
        self.analogs.iter().find(|a| a.name == name)
    }

    /// Adds or replaces an analog input (a 4–20 mA transmitter measuring
    /// a word lane).
    pub fn upsert_analog(&mut self, analog: Analog) -> Result<(), SceneError> {
        let bad = |m: String| {
            Err(SceneError::BadWord(format!(
                "analog `{}`: {m}",
                analog.name
            )))
        };
        if self.signal_readable(&analog.name) || self.words.iter().any(|w| w.name == analog.name) {
            return bad("the name is already a signal, a sensor or a word".into());
        }
        let (lo, hi) = analog.range;
        if !(lo.is_finite() && hi.is_finite()) || lo == hi {
            return bad(format!(
                "the range is the engineering value at 4 mA and at 20 mA — two different numbers, got ({lo}, {hi})"
            ));
        }
        if analog.raw.0 == analog.raw.1 {
            return bad(format!(
                "the counts at 4 mA and at 20 mA must differ, got ({}, {})",
                analog.raw.0, analog.raw.1
            ));
        }
        match self.word_kind(&analog.source) {
            Some(WordKind::Analog) | Some(WordKind::AnalogRaw { .. }) => {
                return bad(format!(
                    "`{}` is an analog input itself — a transmitter measures a word or an encoder",
                    analog.source
                ));
            }
            Some(_) => {}
            None => {
                return bad(format!(
                    "unknown word `{}` to measure (a defined word, `<device>.encoder`)",
                    analog.source
                ));
            }
        }
        self.analogs.retain(|a| a.name != analog.name);
        self.analogs.push(analog);
        Ok(())
    }

    pub fn remove_analog(&mut self, name: &str) -> Result<(), SceneError> {
        let before = self.analogs.len();
        self.analogs.retain(|a| a.name != name);
        if self.analogs.len() == before {
            return Err(SceneError::UnknownWord(name.to_string()));
        }
        Ok(())
    }

    pub fn set_analogs(&mut self, analogs: Vec<Analog>) {
        self.analogs = analogs;
    }

    /// What word lane `name` is, when it is one a program can read.
    pub fn word_kind(&self, name: &str) -> Option<WordKind> {
        if let Some(w) = self.words.iter().find(|w| w.name == name) {
            return Some(WordKind::Defined { integer: w.integer });
        }
        if self.analogs.iter().any(|a| a.name == name) {
            return Some(WordKind::Analog);
        }
        let (head, aspect) = name.rsplit_once('.')?;
        match aspect {
            "raw" => self
                .analogs
                .iter()
                .any(|a| a.name == head)
                .then(|| WordKind::AnalogRaw {
                    analog: head.to_string(),
                }),
            "encoder" => self
                .devices
                .iter()
                .any(|d| {
                    d.name == head
                        && matches!(
                            d.kind,
                            DeviceKind::LinearAxis { encoder: true, .. }
                                | DeviceKind::Conveyor { encoder: true, .. }
                        )
                })
                .then(|| WordKind::Encoder {
                    device: head.to_string(),
                }),
            "id" => self
                .sensors
                .iter()
                .any(|s| s.name == head && s.ids.is_some())
                .then(|| WordKind::Id {
                    sensor: head.to_string(),
                }),
            "count" => self
                .logics
                .iter()
                .flat_map(|l| l.rungs.iter())
                .flat_map(|r| r.outputs.iter())
                .any(|o| matches!(o, RungOutput::Ctu { signal, .. } if signal == head))
                .then(|| WordKind::Count {
                    signal: head.to_string(),
                }),
            _ => None,
        }
    }

    /// Whether an expression over these operands is integer arithmetic:
    /// every operand whole (an integer word, a whole constant).
    pub fn whole(&self, operand: &Operand) -> bool {
        match operand {
            Operand::Const(x) => x.fract() == 0.0,
            Operand::Word(name) => self.word_kind(name).is_some_and(|k| k.integer()),
        }
    }

    fn validate_operand(&self, operand: &Operand) -> Result<(), String> {
        match operand {
            Operand::Const(x) if !x.is_finite() => {
                Err(format!("a constant must be finite, got {x}"))
            }
            Operand::Const(_) => Ok(()),
            Operand::Word(name) => self.word_kind(name).map(|_| ()).ok_or_else(|| {
                format!(
                    "unknown word `{name}` (declare it with define_word, add an analog input, or                      name a recorded word: `<device>.encoder`, `<sensor>.id`, `<counter>.count`)"
                )
            }),
        }
    }

    fn validate_word_write(&self, word: &str, value: &WordExpr) -> Result<(), String> {
        if !self.words.iter().any(|w| w.name == word) {
            return Err(match self.word_kind(word) {
                Some(_) => format!("`{word}` is a measured word — read-only"),
                None => format!("unknown word `{word}` (declare it with define_word)"),
            });
        }
        value
            .operands()
            .try_for_each(|o| self.validate_operand(o))?;
        if let Some((ArithOp::Div, Operand::Const(d))) = &value.op {
            if *d == 0.0 {
                return Err(format!("`{word}`: division by the constant 0"));
            }
        }
        Ok(())
    }

    pub fn sensors(&self) -> &[Sensor] {
        &self.sensors
    }

    /// Adds or replaces a pseudo-sensor. A vision sensor must name an
    /// existing camera, a field sensor an existing lidar — the optics
    /// they judge through.
    pub fn upsert_sensor(&mut self, sensor: Sensor) -> Result<(), SceneError> {
        if let SensorKind::Vision {
            camera,
            detect_range,
            ..
        } = &sensor.kind
        {
            if !self.cameras.iter().any(|c| &c.name == camera) {
                return Err(SceneError::UnknownCamera(camera.clone()));
            }
            if let Some([near, far]) = detect_range {
                if !(*near > 0.0 && far > near) {
                    return Err(SceneError::BadCamera(format!(
                        "vision sensor `{}`: need 0 < near < far, got {near} / {far}",
                        sensor.name
                    )));
                }
            }
        }
        if let SensorKind::Field {
            lidar,
            range,
            sector,
            ..
        } = &sensor.kind
        {
            let Some(scanner) = self.lidars.iter().find(|l| &l.name == lidar) else {
                return Err(SceneError::UnknownLidar(lidar.clone()));
            };
            if let Some(r) = range {
                if !(*r > 0.0 && *r <= scanner.range[1]) {
                    return Err(SceneError::BadLidar(format!(
                        "field sensor `{}`: range must be in (0, {}] (lidar `{lidar}`'s max \
                         range), got {r}",
                        sensor.name, scanner.range[1]
                    )));
                }
            }
            if let Some([start, end]) = sector {
                let half = scanner.fov_deg / 2.0;
                if !(end > start && *start >= -half - 1e-9 && *end <= half + 1e-9) {
                    return Err(SceneError::BadLidar(format!(
                        "field sensor `{}`: sector must satisfy -{half} <= start < end <= \
                         {half} (lidar `{lidar}`'s sweep), got [{start}, {end}]",
                        sensor.name
                    )));
                }
            }
        }
        match self.sensors.iter_mut().find(|s| s.name == sensor.name) {
            Some(slot) => *slot = sensor,
            None => self.sensors.push(sensor),
        }
        Ok(())
    }

    pub fn remove_sensor(&mut self, name: &str) -> Result<(), SceneError> {
        let before = self.sensors.len();
        self.sensors.retain(|s| s.name != name);
        if self.sensors.len() == before {
            return Err(SceneError::UnknownSensor(name.to_string()));
        }
        self.prune_parts();
        Ok(())
    }

    pub fn set_sensors(&mut self, sensors: Vec<Sensor>) {
        self.sensors = sensors;
        self.prune_parts();
    }

    pub fn cameras(&self) -> &[Camera] {
        &self.cameras
    }

    /// Adds or replaces a camera, validating its mount and optics — a
    /// camera aimed from a link that does not exist, or with a degenerate
    /// frustum, is an authoring mistake worth catching at the call site.
    pub fn upsert_camera(&mut self, camera: Camera) -> Result<(), SceneError> {
        match &camera.mount {
            CameraMount::World => {}
            CameraMount::Vehicle { device } => {
                if !self.devices.iter().any(|d| &d.name == device) {
                    return Err(SceneError::UnknownDevice(device.clone()));
                }
            }
            CameraMount::Link { robot, link } => {
                let Some(r) = self.robot_index(robot) else {
                    return Err(SceneError::UnknownRobot(robot.clone()));
                };
                if self.robots()[r].model.link_index(link).is_none() {
                    return Err(SceneError::UnknownLink(link.clone()));
                }
            }
        }
        if !(camera.fov_deg > 0.0 && camera.fov_deg < 180.0) {
            return Err(SceneError::BadCamera(format!(
                "camera `{}`: fov must be in (0, 180) degrees, got {}",
                camera.name, camera.fov_deg
            )));
        }
        if camera.resolution[0] == 0 || camera.resolution[1] == 0 {
            return Err(SceneError::BadCamera(format!(
                "camera `{}`: resolution must be positive, got {}x{}",
                camera.name, camera.resolution[0], camera.resolution[1]
            )));
        }
        if !(camera.near > 0.0 && camera.far > camera.near) {
            return Err(SceneError::BadCamera(format!(
                "camera `{}`: need 0 < near < far, got {} / {}",
                camera.name, camera.near, camera.far
            )));
        }
        match self.cameras.iter_mut().find(|c| c.name == camera.name) {
            Some(slot) => *slot = camera,
            None => self.cameras.push(camera),
        }
        Ok(())
    }

    pub fn remove_camera(&mut self, name: &str) -> Result<(), SceneError> {
        // A vision sensor looking through it would go blind silently.
        if let Some(sensor) = self
            .sensors
            .iter()
            .find(|s| matches!(&s.kind, SensorKind::Vision { camera, .. } if camera == name))
        {
            return Err(SceneError::BadCamera(format!(
                "camera `{name}` is watched by vision sensor `{}`; remove the sensor first",
                sensor.name
            )));
        }
        let before = self.cameras.len();
        self.cameras.retain(|c| c.name != name);
        if self.cameras.len() == before {
            return Err(SceneError::UnknownCamera(name.to_string()));
        }
        self.prune_parts();
        Ok(())
    }

    /// Wholesale replacement (project load) — trusted, like
    /// [`Scene::set_sensors`].
    pub fn set_cameras(&mut self, cameras: Vec<Camera>) {
        self.cameras = cameras;
        self.prune_parts();
    }

    pub fn lidars(&self) -> &[Lidar] {
        &self.lidars
    }

    /// Adds or replaces a LiDAR scanner, validating its mount and sweep —
    /// a scanner bolted to a link that does not exist, or with a
    /// degenerate sector, is an authoring mistake worth catching at the
    /// call site.
    pub fn upsert_lidar(&mut self, lidar: Lidar) -> Result<(), SceneError> {
        match &lidar.mount {
            LidarMount::World => {}
            LidarMount::Vehicle { device } => {
                if !self.devices.iter().any(|d| &d.name == device) {
                    return Err(SceneError::UnknownDevice(device.clone()));
                }
            }
            LidarMount::Link { robot, link } => {
                let Some(r) = self.robot_index(robot) else {
                    return Err(SceneError::UnknownRobot(robot.clone()));
                };
                if self.robots()[r].model.link_index(link).is_none() {
                    return Err(SceneError::UnknownLink(link.clone()));
                }
            }
        }
        if !(lidar.fov_deg > 0.0 && lidar.fov_deg <= 360.0) {
            return Err(SceneError::BadLidar(format!(
                "lidar `{}`: fov must be in (0, 360] degrees, got {}",
                lidar.name, lidar.fov_deg
            )));
        }
        let [min, max] = lidar.range;
        if !(min > 0.0 && max > min) {
            return Err(SceneError::BadLidar(format!(
                "lidar `{}`: need 0 < min range < max range, got {min} / {max}",
                lidar.name
            )));
        }
        if !(lidar.resolution_deg > 0.0 && lidar.resolution_deg <= lidar.fov_deg) {
            return Err(SceneError::BadLidar(format!(
                "lidar `{}`: resolution must be in (0, fov] degrees, got {}",
                lidar.name, lidar.resolution_deg
            )));
        }
        if lidar.channels == 0 {
            return Err(SceneError::BadLidar(format!(
                "lidar `{}`: channels must be >= 1 (1 = a planar scanner)",
                lidar.name
            )));
        }
        if lidar.channels > 1 && !(lidar.vfov_deg > 0.0 && lidar.vfov_deg < 180.0) {
            return Err(SceneError::BadLidar(format!(
                "lidar `{}`: a {}-channel scanner needs vfov in (0, 180) degrees, got {}",
                lidar.name, lidar.channels, lidar.vfov_deg
            )));
        }
        if lidar.channels == 1 && lidar.vfov_deg != 0.0 {
            return Err(SceneError::BadLidar(format!(
                "lidar `{}`: vfov needs channels >= 2 (a planar scanner has no \
                 vertical field), got channels=1, vfov={}",
                lidar.name, lidar.vfov_deg
            )));
        }
        match self.lidars.iter_mut().find(|l| l.name == lidar.name) {
            Some(slot) => *slot = lidar,
            None => self.lidars.push(lidar),
        }
        Ok(())
    }

    pub fn remove_lidar(&mut self, name: &str) -> Result<(), SceneError> {
        // A field sensor sweeping through it would go blind silently.
        if let Some(sensor) = self
            .sensors
            .iter()
            .find(|s| matches!(&s.kind, SensorKind::Field { lidar, .. } if lidar == name))
        {
            return Err(SceneError::BadLidar(format!(
                "lidar `{name}` is swept by field sensor `{}`; remove the sensor first",
                sensor.name
            )));
        }
        let before = self.lidars.len();
        self.lidars.retain(|l| l.name != name);
        if self.lidars.len() == before {
            return Err(SceneError::UnknownLidar(name.to_string()));
        }
        self.prune_parts();
        Ok(())
    }

    /// Wholesale replacement (project load) — trusted, like
    /// [`Scene::set_sensors`].
    pub fn set_lidars(&mut self, lidars: Vec<Lidar>) {
        self.lidars = lidars;
        self.prune_parts();
    }

    pub fn devices(&self) -> &[Device] {
        &self.devices
    }

    /// Adds or replaces an auxiliary device.
    /// Declares a weld flash bound to `signal` at `robot`'s TCP. The
    /// signal must already exist (an internal signal or a sensor input) —
    /// a flash nobody can drive is an authoring mistake, not a display
    /// preference. Re-declaring a name replaces it.
    pub fn add_weld_flash(
        &mut self,
        name: &str,
        signal: &str,
        robot: &str,
    ) -> Result<(), SceneError> {
        if self.robot_index(robot).is_none() {
            return Err(SceneError::UnknownRobot(robot.to_string()));
        }
        if !self.signals.iter().any(|s| s.name == signal)
            && !self.sensors.iter().any(|s| s.name == signal)
        {
            return Err(SceneError::UnknownSignal(signal.to_string()));
        }
        let flash = WeldFlash {
            name: name.to_string(),
            signal: signal.to_string(),
            robot: robot.to_string(),
            kind: FlashKind::Flash,
            spin_link: None,
            cone: None,
            obstacle: None,
            color: None,
        };
        match self.weld_flashes.iter_mut().find(|f| f.name == name) {
            Some(slot) => *slot = flash,
            None => self.weld_flashes.push(flash),
        }
        Ok(())
    }

    /// Binds an accumulating cut-trace effect to `signal` at `robot`'s
    /// TCP, optionally spinning `spin_link` while the signal is on. Same
    /// contract as [`Scene::add_weld_flash`]: presentation only, driven
    /// by the baked signal, zero effect on the cycle.
    pub fn add_cut_trace(
        &mut self,
        name: &str,
        signal: &str,
        robot: &str,
        spin_link: Option<&str>,
    ) -> Result<(), SceneError> {
        let Some(r) = self.robot_index(robot) else {
            return Err(SceneError::UnknownRobot(robot.to_string()));
        };
        if !self.signals.iter().any(|s| s.name == signal)
            && !self.sensors.iter().any(|s| s.name == signal)
        {
            return Err(SceneError::UnknownSignal(signal.to_string()));
        }
        if let Some(link) = spin_link {
            if self.robots()[r].model.link_index(link).is_none() {
                return Err(SceneError::UnknownLink(link.to_string()));
            }
        }
        let flash = WeldFlash {
            name: name.to_string(),
            signal: signal.to_string(),
            robot: robot.to_string(),
            kind: FlashKind::Trace,
            spin_link: spin_link.map(str::to_string),
            cone: None,
            obstacle: None,
            color: None,
        };
        match self.weld_flashes.iter_mut().find(|f| f.name == name) {
            Some(slot) => *slot = flash,
            None => self.weld_flashes.push(flash),
        }
        Ok(())
    }

    /// Binds a spray-cone effect to `signal` at `robot`'s TCP: a
    /// translucent cone `length` long and `radius` wide at its base,
    /// pointing along the TCP's -Z (the spray direction) while the signal
    /// is on. Same contract as [`Scene::add_weld_flash`]: presentation
    /// only, driven by the baked signal, zero effect on the cycle.
    pub fn add_spray_cone(
        &mut self,
        name: &str,
        signal: &str,
        robot: &str,
        length: f64,
        radius: f64,
    ) -> Result<(), SceneError> {
        if self.robot_index(robot).is_none() {
            return Err(SceneError::UnknownRobot(robot.to_string()));
        }
        if !self.signals.iter().any(|s| s.name == signal)
            && !self.sensors.iter().any(|s| s.name == signal)
        {
            return Err(SceneError::UnknownSignal(signal.to_string()));
        }
        if !(length > 0.0 && radius > 0.0) {
            return Err(SceneError::BadEffect(format!(
                "spray cone `{name}`: length and radius must be positive, got {length} / {radius}"
            )));
        }
        let flash = WeldFlash {
            name: name.to_string(),
            signal: signal.to_string(),
            robot: robot.to_string(),
            kind: FlashKind::Spray,
            spin_link: None,
            cone: Some(SprayCone { length, radius }),
            obstacle: None,
            color: None,
        };
        match self.weld_flashes.iter_mut().find(|f| f.name == name) {
            Some(slot) => *slot = flash,
            None => self.weld_flashes.push(flash),
        }
        Ok(())
    }

    /// Binds a spin effect to `signal`: `link` of `robot` turns about its
    /// own Z while the signal is on, nothing else drawn — a screwdriver's
    /// bit running a screw down. Same contract as [`Scene::add_weld_flash`]:
    /// presentation only, driven by the baked signal, zero effect on the
    /// cycle; USD export carries nothing for it.
    pub fn add_spin(
        &mut self,
        name: &str,
        signal: &str,
        robot: &str,
        link: &str,
    ) -> Result<(), SceneError> {
        let Some(r) = self.robot_index(robot) else {
            return Err(SceneError::UnknownRobot(robot.to_string()));
        };
        if !self.signals.iter().any(|s| s.name == signal)
            && !self.sensors.iter().any(|s| s.name == signal)
        {
            return Err(SceneError::UnknownSignal(signal.to_string()));
        }
        if self.robots()[r].model.link_index(link).is_none() {
            return Err(SceneError::UnknownLink(link.to_string()));
        }
        let flash = WeldFlash {
            name: name.to_string(),
            signal: signal.to_string(),
            robot: robot.to_string(),
            kind: FlashKind::Spin,
            spin_link: Some(link.to_string()),
            cone: None,
            obstacle: None,
            color: None,
        };
        match self.weld_flashes.iter_mut().find(|f| f.name == name) {
            Some(slot) => *slot = flash,
            None => self.weld_flashes.push(flash),
        }
        Ok(())
    }

    /// Binds an indicator lamp to `signal`: `obstacle` is lit in `color`
    /// (linear RGB) while the signal — an internal signal or a sensor's
    /// lane — is on, and drawn as authored otherwise. Presentation only,
    /// like [`Scene::add_weld_flash`]: the lamp a PLC drives with an
    /// output, shown the way the operator sees it.
    pub fn add_lamp(
        &mut self,
        name: &str,
        signal: &str,
        obstacle: &str,
        color: [f32; 3],
    ) -> Result<(), SceneError> {
        if !self.signals.iter().any(|s| s.name == signal)
            && !self.sensors.iter().any(|s| s.name == signal)
        {
            return Err(SceneError::UnknownSignal(signal.to_string()));
        }
        if !self.obstacles.iter().any(|o| o.name == obstacle) {
            return Err(SceneError::UnknownObstacle(obstacle.to_string()));
        }
        if color.iter().any(|c| !(0.0..=1.0).contains(c)) {
            return Err(SceneError::BadEffect(format!(
                "lamp `{name}`: colour components must be within 0..1, got {color:?}"
            )));
        }
        let flash = WeldFlash {
            name: name.to_string(),
            signal: signal.to_string(),
            robot: String::new(),
            kind: FlashKind::Lamp,
            spin_link: None,
            cone: None,
            obstacle: Some(obstacle.to_string()),
            color: Some(color),
        };
        match self.weld_flashes.iter_mut().find(|f| f.name == name) {
            Some(slot) => *slot = flash,
            None => self.weld_flashes.push(flash),
        }
        Ok(())
    }

    pub fn weld_flashes(&self) -> &[WeldFlash] {
        &self.weld_flashes
    }

    /// Wholesale replacement (project load).
    pub fn set_weld_flashes(&mut self, flashes: Vec<WeldFlash>) {
        self.weld_flashes = flashes;
    }

    pub fn upsert_device(&mut self, device: Device) {
        match self.devices.iter_mut().find(|d| d.name == device.name) {
            Some(slot) => *slot = device,
            None => self.devices.push(device),
        }
    }

    pub fn remove_device(&mut self, name: &str) -> Result<(), SceneError> {
        let before = self.devices.len();
        self.devices.retain(|d| d.name != name);
        if self.devices.len() == before {
            return Err(SceneError::UnknownDevice(name.to_string()));
        }
        self.prune_parts();
        Ok(())
    }

    pub fn set_devices(&mut self, devices: Vec<Device>) {
        self.devices = devices;
        self.prune_parts();
    }

    // ----------------------------------------------------------- scenarios

    pub fn scenarios(&self) -> &[Scenario] {
        &self.scenarios
    }

    pub fn scenario(&self, name: &str) -> Option<&Scenario> {
        self.scenarios.iter().find(|s| s.name == name)
    }

    /// Adds or replaces a scenario wholesale. `baseline` is the reserved
    /// name of the unmodified scene and cannot be defined.
    pub fn upsert_scenario(&mut self, scenario: Scenario) -> Result<(), SceneError> {
        if scenario.name == BASELINE_SCENARIO {
            return Err(SceneError::BadScenario(format!(
                "`{BASELINE_SCENARIO}` is the reserved name of the unmodified scene"
            )));
        }
        match self.scenarios.iter_mut().find(|s| s.name == scenario.name) {
            Some(slot) => *slot = scenario,
            None => self.scenarios.push(scenario),
        }
        Ok(())
    }

    pub fn remove_scenario(&mut self, name: &str) -> Result<(), SceneError> {
        let before = self.scenarios.len();
        self.scenarios.retain(|s| s.name != name);
        if self.scenarios.len() == before {
            return Err(SceneError::UnknownScenario(name.to_string()));
        }
        Ok(())
    }

    pub fn set_scenarios(&mut self, scenarios: Vec<Scenario>) {
        self.scenarios = scenarios;
    }

    /// Applies a scenario's deltas to this scene — meant for the snapshot
    /// a rollout is about to run on, never the live scene. `baseline`
    /// applies nothing. Everything is validated before anything is
    /// touched, so a failed apply leaves the scene unchanged.
    pub fn apply_scenario(&mut self, name: &str) -> Result<(), SceneError> {
        if name == BASELINE_SCENARIO {
            return Ok(());
        }
        let scenario = self
            .scenario(name)
            .ok_or_else(|| SceneError::UnknownScenario(name.to_string()))?
            .clone();
        for (signal, _) in &scenario.signals {
            if !self.signals.iter().any(|s| &s.name == signal) {
                return Err(SceneError::BadScenario(format!(
                    "scenario `{name}`: unknown signal `{signal}` (declare it with \
                     define_signal; sensors are geometric and follow from the world)"
                )));
            }
        }
        for (obstacle, _) in &scenario.obstacles {
            self.obstacle_index(obstacle).map_err(|_| {
                SceneError::BadScenario(format!("scenario `{name}`: unknown obstacle `{obstacle}`"))
            })?;
            // Moving an attached obstacle re-grasps it — a live-editing
            // gesture, not a world variation. Refuse rather than surprise.
            if self.attachments().iter().any(|a| &a.object == obstacle) {
                return Err(SceneError::BadScenario(format!(
                    "scenario `{name}`: `{obstacle}` is attached — a scenario \
                     varies the world's starting state, not a held object \
                     (detach it, or vary the grasp in the sequence)"
                )));
            }
        }
        let forced = self.resolve_faults(name, &scenario.faults)?;
        let mut joint_targets = Vec::new();
        for (robot, positions) in &scenario.joints {
            let index = self
                .robot_index(robot)
                .ok_or_else(|| SceneError::UnknownRobot(robot.clone()))?;
            let expected = self.robots()[index].model.dof();
            if positions.len() != expected {
                return Err(SceneError::WrongDof {
                    expected,
                    got: positions.len(),
                });
            }
            joint_targets.push((index, positions.clone()));
        }
        for (signal, value) in &scenario.signals {
            self.define_signal(signal, *value);
        }
        for (obstacle, pose) in &scenario.obstacles {
            self.set_obstacle_pose(obstacle, *pose)?;
        }
        for (index, positions) in joint_targets {
            self.set_joint_positions_for(index, positions)?;
        }
        self.forced_inputs = forced;
        Ok(())
    }

    /// Checks a scenario's faults against the scene and resolves each to
    /// the value its lane is pinned to. A target must be a sensor or an
    /// internal signal (things with an input lane); `Open` needs an input
    /// *wire* — a sensor, or a signal some program reads without also
    /// writing it on the same controller — and takes the polarity of the
    /// point's binding (`invert`), `false` unbound.
    pub(crate) fn resolve_faults(
        &self,
        scenario: &str,
        faults: &[Fault],
    ) -> Result<Vec<(String, bool)>, SceneError> {
        let mut out: Vec<(String, bool)> = Vec::new();
        let mut derivation: Option<crate::iomap::IoDerivation> = None;
        for fault in faults {
            let target = fault.target.as_str();
            if fault.kind == FaultKind::NodeDown {
                // The node and everything hanging off it: each input lane
                // bound there opens with its own wire's polarity.
                if self.io.node(target).is_none() {
                    return Err(SceneError::BadScenario(format!(
                        "scenario `{scenario}`: `{target}` is not an I/O node (add_io_node) — node_down takes a controller or a station"
                    )));
                }
                let mut opened = 0usize;
                for b in &self.io.bindings {
                    if b.point.direction != crate::iomap::IoDirection::Input
                        || b.point.aspect.is_some()
                        || !self.io.reach(&b.node).iter().any(|r| r == target)
                    {
                        continue;
                    }
                    let name = b.point.name.as_str();
                    let has_lane = self.sensors.iter().any(|s| s.name == name)
                        || self.signals.iter().any(|s| s.name == name);
                    if !has_lane {
                        continue; // robot done / device done: no lane to force
                    }
                    if out.iter().any(|(t, _)| t == name) {
                        return Err(SceneError::BadScenario(format!(
                            "scenario `{scenario}`: `{name}` is forced twice (node_down `{target}` opens it too)"
                        )));
                    }
                    out.push((name.to_string(), b.invert));
                    opened += 1;
                }
                if opened == 0 {
                    return Err(SceneError::BadScenario(format!(
                        "scenario `{scenario}`: node_down `{target}` opens nothing — no sensor or signal input is bound on it (or its stations)"
                    )));
                }
                continue;
            }
            // An analog input: its loop opens (0 mA); it has no stuck level.
            if self.analogs.iter().any(|a| a.name == target) {
                if fault.kind != FaultKind::Open {
                    return Err(SceneError::BadScenario(format!(
                        "scenario `{scenario}`: `{target}` is an analog input — open its loop \
                         (fault kind `open`); a stuck contact has no meaning on a 4–20 mA signal"
                    )));
                }
                if out.iter().any(|(t, _)| t == target) {
                    return Err(SceneError::BadScenario(format!(
                        "scenario `{scenario}`: `{target}` is forced twice"
                    )));
                }
                out.push((target.to_string(), false));
                continue;
            }
            let is_sensor = self.sensors.iter().any(|s| s.name == target);
            let is_signal = self.signals.iter().any(|s| s.name == target);
            if !(is_sensor || is_signal) {
                let hint = if self.devices.iter().any(|d| d.name == target) {
                    "a device's running lane is an output; a fault forces an input"
                } else if self.robots.iter().any(|r| r.name == target) {
                    "a robot has no input lane; force the signal its program waits on"
                } else {
                    "a fault forces a sensor or an internal signal (define_signal)"
                };
                return Err(SceneError::BadScenario(format!(
                    "scenario `{scenario}`: fault target `{target}` is not a sensor or an internal signal — {hint}"
                )));
            }
            if out.iter().any(|(t, _)| t == target) {
                return Err(SceneError::BadScenario(format!(
                    "scenario `{scenario}`: `{target}` is forced twice"
                )));
            }
            let value = match fault.kind {
                FaultKind::StuckAt(value) => value,
                FaultKind::NodeDown => unreachable!("handled above"),
                FaultKind::Open => {
                    if is_signal {
                        // Read-only and handshake inputs have a wire to
                        // open; a relay written and read on one controller
                        // (or only written) has none.
                        let d = match &derivation {
                            Some(d) => d,
                            None => {
                                derivation =
                                    Some(crate::iomap::derive(self, None).map_err(|e| {
                                        SceneError::BadScenario(format!(
                                            "scenario `{scenario}`: {e}"
                                        ))
                                    })?);
                                derivation.as_ref().unwrap()
                            }
                        };
                        let has_input = d.points.iter().any(|p| {
                            p.id.name == target
                                && p.id.aspect.is_none()
                                && p.id.direction == crate::iomap::IoDirection::Input
                        });
                        if !has_input {
                            return Err(SceneError::BadScenario(format!(
                                "scenario `{scenario}`: `{target}` has no input wire to open — it is written and read on one controller (or only written); force it with stuck(...) instead"
                            )));
                        }
                    }
                    self.io
                        .bindings
                        .iter()
                        .find(|b| {
                            b.point.name == target
                                && b.point.aspect.is_none()
                                && b.point.direction == crate::iomap::IoDirection::Input
                        })
                        .is_some_and(|b| b.invert)
                }
            };
            out.push((target.to_string(), value));
        }
        Ok(out)
    }

    /// The inputs the applied scenario pins, `(lane name, value)` — set by
    /// [`apply_scenario`](Self::apply_scenario) on the snapshot a rollout
    /// runs on, empty on a live scene. Not authored, not saved.
    pub fn forced_inputs(&self) -> &[(String, bool)] {
        &self.forced_inputs
    }

    /// Is `name` readable as a signal (internal relay or sensor input)?
    fn signal_readable(&self, name: &str) -> bool {
        self.signals.iter().any(|s| s.name == name)
            || self.sensors.iter().any(|s| s.name == name)
            || self.devices.iter().any(|d| {
                // An axis's stop lanes read like sensors: `<axis>/<stop>`.
                matches!(&d.kind, DeviceKind::LinearAxis { stops, .. }
                    if stops.iter().any(|(stop, _)| format!("{}/{}", d.name, stop) == name))
            })
    }

    /// Resolves an action's robot reference to a scene index. `None` means
    /// the sole robot; with several robots it is ambiguous and rejected
    /// (mirroring the Python API rule).
    pub(crate) fn resolve_seq_robot(&self, robot: &Option<String>) -> Result<usize, String> {
        let names = || {
            self.robots()
                .iter()
                .map(|r| format!("`{}`", r.name))
                .collect::<Vec<_>>()
                .join(", ")
        };
        match robot {
            Some(name) => self
                .robot_index(name)
                .ok_or_else(|| format!("unknown robot `{name}` (robots: {})", names())),
            None if self.robots().len() == 1 => Ok(0),
            None => Err(format!(
                "the scene has {} robots; give the action a robot (one of: {})",
                self.robots().len(),
                names()
            )),
        }
    }

    /// The robot and joints a driver action moves: a motion's owner and
    /// its group's joints, a ramp's addressed robot and target joints, a
    /// toolpath's robot and every joint. `None` for non-driver actions.
    fn driver_joints(&self, action: &Action) -> Result<Option<(usize, Vec<usize>)>, String> {
        match action {
            Action::StartMotion { motion } => Ok(self
                .motions
                .iter()
                .find(|m| &m.name == motion)
                .map(|m| (m.robot, self.motion_joints(m)))),
            Action::StartRamp { robot, targets, .. } => {
                let r = self.resolve_seq_robot(robot)?;
                let model = &self.robots()[r].model;
                let joints = targets
                    .iter()
                    .filter_map(|(joint, _)| {
                        model
                            .joint_index(joint)
                            .and_then(|ji| model.joints[ji].q_index)
                    })
                    .collect();
                Ok(Some((r, joints)))
            }
            Action::StartToolpath { robot, .. } => {
                let r = self.resolve_seq_robot(robot)?;
                Ok(Some((r, (0..self.robots()[r].model.dof()).collect())))
            }
            Action::Policy { robot, group, .. } => {
                let r = self.resolve_seq_robot(robot)?;
                let g = match group {
                    Some(name) => Some(
                        self.robots()[r]
                            .model
                            .group_index(name)
                            .ok_or_else(|| format!("unknown group `{name}`"))?,
                    ),
                    None => None,
                };
                Ok(Some((r, self.group_joints(r, g))))
            }
            // A crouch drives the legs (the arms ride the body).
            Action::Crouch { robot, .. } => {
                let r = self.resolve_seq_robot(robot)?;
                Ok(Some((r, self.crouch_legs(r)?)))
            }
            _ => Ok(None),
        }
    }

    /// The legs a crouch of robot `r` moves: its mounted gait's leg DOF.
    /// An error names the robot when it has no gait to crouch on.
    pub(crate) fn crouch_legs(&self, r: usize) -> Result<Vec<usize>, String> {
        let robot = &self.robots()[r];
        let spec = robot
            .mount
            .as_ref()
            .and_then(|m| m.gait.as_ref())
            .ok_or_else(|| {
                format!(
                    "`{}` has no gait to crouch on — crouch is for a legged robot mounted \
                     with `gait=` (mount_robot)",
                    robot.name
                )
            })?;
        crate::gait::resolve_gait(&robot.model, spec, robot.joint_positions())
            .map(|g| g.leg_joints())
            .map_err(|m| format!("`{}`: gait: {m}", robot.name))
    }

    /// The joints a motion drives: its group's, or every joint of its
    /// robot (see [`crate::motion::motion_group`]).
    pub(crate) fn motion_joints(&self, motion: &crate::motion::Motion) -> Vec<usize> {
        let model = &self.robots()[motion.robot].model;
        match crate::motion::motion_group(model, motion) {
            Some(g) => g.joints,
            None => (0..model.dof()).collect(),
        }
    }

    /// Single-owner check for concurrently-running programs: every robot,
    /// device, and *written* signal may be commanded by at most one of the
    /// given sequences. Reading is free — conditions (`robot_done`,
    /// `signal`, `device_done`) are how programs watch each other — but
    /// two writers on one resource is the arbitration problem PLCs solve
    /// by not allowing it, and so do we: it is rejected at authoring time
    /// rather than refereed at runtime.
    pub(crate) fn validate_program_ownership(&self, sequences: &[Sequence]) -> Result<(), String> {
        use std::collections::HashMap;
        // resource key (kind, display name) -> owning sequence index
        let mut owners: HashMap<(u8, String), usize> = HashMap::new();
        let conflict = |owners: &HashMap<(u8, String), usize>,
                        kind: u8,
                        label: &str,
                        name: &str,
                        program: usize|
         -> Result<(), String> {
            match owners.get(&(kind, name.to_string())) {
                Some(&owner) if owner != program => Err(format!(
                    "{label} `{name}` is commanded by both `{}` and `{}`; every robot, \
                     device, and written signal belongs to one program (programs watch \
                     each other through conditions, not by sharing outputs)",
                    sequences[owner].name, sequences[program].name,
                )),
                _ => Ok(()),
            }
        };
        let claim = |owners: &mut HashMap<(u8, String), usize>,
                     kind: u8,
                     label: &str,
                     name: String,
                     program: usize|
         -> Result<(), String> {
            conflict(owners, kind, label, &name, program)?;
            owners.insert((kind, name), program);
            Ok(())
        };
        // A dual-arm robot's arms are resources of their own: two programs
        // may drive one robot, an arm each (`robot/arm`). An action that
        // names no arm on such a robot claims the whole robot, which then
        // may be nobody else's — arms included.
        let claim_robot = |owners: &mut HashMap<(u8, String), usize>,
                           robot: usize,
                           arm: Option<String>,
                           program: usize|
         -> Result<(), String> {
            let r = &self.robots()[robot];
            let arms = r.model.groups();
            if arms.len() > 1 {
                if let Some(arm) = arm {
                    conflict(owners, 0, "robot", &r.name, program)?;
                    return claim(owners, 0, "robot", format!("{}/{arm}", r.name), program);
                }
                for other in &arms {
                    let name = format!("{}/{}", r.name, other.name);
                    conflict(owners, 0, "robot", &name, program)?;
                }
            }
            claim(owners, 0, "robot", r.name.clone(), program)
        };
        for (index, sequence) in sequences.iter().enumerate() {
            // Rungs own what they drive as steps do.
            let steps = self.program_steps(sequence);
            walk_actions(&steps, &mut |action| -> Result<(), String> {
                match action {
                    Action::StartMotion { motion } => {
                        // Validation has already established the motion
                        // exists; its robot (or arm) is the claimed resource.
                        if let Some(m) = self.motions().iter().find(|m| &m.name == motion) {
                            let arm = m
                                .group
                                .and_then(|g| {
                                    self.robots()[m.robot].model.groups().into_iter().nth(g)
                                })
                                .map(|g| g.name);
                            claim_robot(&mut owners, m.robot, arm, index)?;
                        }
                    }
                    Action::StartRamp { robot, targets, .. } => {
                        if let Ok(r) = self.resolve_seq_robot(robot) {
                            let joints: Vec<&str> =
                                targets.iter().map(|(j, _)| j.as_str()).collect();
                            let arm = self.ramp_arm(r, &joints);
                            claim_robot(&mut owners, r, arm, index)?;
                        }
                    }
                    Action::Policy { robot, group, .. } => {
                        if let Ok(r) = self.resolve_seq_robot(robot) {
                            claim_robot(&mut owners, r, group.clone(), index)?;
                        }
                    }
                    Action::Attach { robot, group, .. }
                    | Action::Track { robot, group, .. }
                    | Action::Untrack { robot, group } => {
                        if let Ok(r) = self.resolve_seq_robot(robot) {
                            claim_robot(&mut owners, r, group.clone(), index)?;
                        }
                    }
                    Action::StartToolpath { robot, .. } | Action::Crouch { robot, .. } => {
                        if let Ok(r) = self.resolve_seq_robot(robot) {
                            claim_robot(&mut owners, r, None, index)?;
                        }
                    }
                    // Detach names an object; the carrying robot is
                    // whoever attached it, which the same-owner rule
                    // for Attach already pins to one program.
                    Action::Detach { .. } => {}
                    Action::Set { signal, .. } => {
                        claim(&mut owners, 1, "signal", signal.clone(), index)?;
                    }
                    Action::SetWord { word, .. } => {
                        claim(&mut owners, 3, "word", word.clone(), index)?;
                    }
                    Action::Device { device, .. } => {
                        claim(&mut owners, 2, "device", device.clone(), index)?;
                    }
                }
                Ok(())
            })?;
        }
        Ok(())
    }

    /// The arm of a dual-arm robot a ramp drives: the group every target
    /// joint belongs to. `None` when the robot has one group, the joints
    /// straddle arms, or none is actuated.
    pub(crate) fn ramp_arm(&self, robot: usize, joints: &[&str]) -> Option<String> {
        let model = &self.robots()[robot].model;
        let groups = model.groups();
        if groups.len() < 2 {
            return None;
        }
        let qs: Vec<usize> = joints
            .iter()
            .filter_map(|j| model.joint_index(j).and_then(|ji| model.joints[ji].q_index))
            .collect();
        if qs.is_empty() {
            return None;
        }
        groups
            .into_iter()
            .find(|g| qs.iter().all(|q| g.joints.contains(q)))
            .map(|g| g.name)
    }

    /// Full authoring-time validation, run before a rollout. `Err` carries
    /// the offending step index (when applicable) and a message.
    pub(crate) fn validate_sequence(
        &self,
        sequence: &Sequence,
    ) -> Result<(), (Option<usize>, String)> {
        let logic = self.logic(&sequence.name);
        if sequence.steps.is_empty() && logic.is_none_or(|l| l.rungs.is_empty()) {
            return Err((None, "sequence has no steps".to_string()));
        }
        if let Some(logic) = logic {
            self.validate_logic(logic).map_err(|m| (None, m))?;
        }
        for sensor in &self.sensors {
            if self.signals.iter().any(|s| s.name == sensor.name) {
                return Err((
                    None,
                    format!(
                        "sensor `{}` collides with an internal signal of the same name",
                        sensor.name
                    ),
                ));
            }
        }
        for device in &self.devices {
            self.validate_vehicle(device).map_err(|m| (None, m))?;
            self.validate_formation(device).map_err(|m| (None, m))?;
            self.validate_lift(device).map_err(|m| (None, m))?;
            self.validate_axis(device).map_err(|m| (None, m))?;
        }
        // A gait is checked against the model at mount time; what it cannot
        // know then is how fast the vehicle it rides will drive.
        for robot in self.robots() {
            let Some(mount) = &robot.mount else { continue };
            let Some(spec) = &mount.gait else { continue };
            let resolved = crate::gait::resolve_gait(&robot.model, spec, robot.joint_positions())
                .map_err(|m| (None, format!("robot `{}` gait: {m}", robot.name)))?;
            let rates = self.devices.iter().find_map(|d| match &d.kind {
                DeviceKind::Vehicle {
                    speed,
                    turn_speed,
                    drive,
                    ..
                } if d.name == mount.device => {
                    Some((*speed, *turn_speed, matches!(drive, Drive::Aerial { .. })))
                }
                _ => None,
            });
            let Some((speed, turn_speed, aerial)) = rates else {
                return Err((
                    None,
                    format!(
                        "robot `{}` is mounted on `{}`, which is not a vehicle",
                        robot.name, mount.device
                    ),
                ));
            };
            if aerial {
                return Err((
                    None,
                    format!(
                        "robot `{}` has a gait, but `{}` is an aerial vehicle — legs \
                         walk floors; give the machine a ground drive or drop the gait",
                        robot.name, mount.device
                    ),
                ));
            }
            // A wheel-legged machine set to roll never strides: the
            // vehicle's rates are its rolling rates, and the legs sit out.
            // Deciding leg by leg, it walks at the gait's own pace.
            let rates = match mount.drive.as_ref().map(|d| d.mode) {
                Some(LocomotionMode::Roll) => None,
                Some(LocomotionMode::Auto) => Some(spec.walk_rates(speed, turn_speed)),
                Some(LocomotionMode::Walk) | None => Some((speed, turn_speed)),
            };
            if let Some((speed, turn_speed)) = rates {
                crate::gait::check_stride(&resolved, &mount.offset, speed, turn_speed)
                    .map_err(|m| (None, format!("robot `{}`: {m}", robot.name)))?;
            }
        }
        for sensor in &self.sensors {
            let Some(mount) = &sensor.mount else { continue };
            match self.devices.iter().find(|d| &d.name == mount) {
                Some(d) if matches!(d.kind, DeviceKind::Vehicle { .. }) => {}
                Some(_) => {
                    return Err((
                        None,
                        format!(
                            "sensor `{}` is mounted on `{mount}`, which is not a vehicle \
                             (only vehicles carry sensors)",
                            sensor.name
                        ),
                    ))
                }
                None => {
                    return Err((
                        None,
                        format!(
                            "sensor `{}` is mounted on unknown device `{mount}`",
                            sensor.name
                        ),
                    ))
                }
            }
        }
        // Tracking and grasping are modes the steps switch on and off, so
        // their rules are checked by walking the step list once — per
        // robot for tracking, per object for grasps. Branch arms fork
        // that state; every arm must rejoin in the same state or the
        // steps after the join cannot be validated (or executed) sanely.
        let mut tracked: Vec<Vec<TrackClaim>> = vec![Vec::new(); self.robots().len()];
        let mut held: std::collections::HashMap<String, (usize, usize)> = self
            .attachments()
            .iter()
            .map(|a| (a.object.clone(), (a.robot, a.link)))
            .collect();
        self.validate_steps(&sequence.steps, &mut tracked, &mut held)
    }

    /// The per-step walk of [`validate_sequence`], recursing into branch
    /// arms. Errors carry the local step index; branch recursion rewraps
    /// them onto the branching step with the arm named.
    fn validate_steps(
        &self,
        steps: &[Step],
        tracked: &mut Vec<Vec<TrackClaim>>,
        held: &mut std::collections::HashMap<String, (usize, usize)>,
    ) -> Result<(), (Option<usize>, String)> {
        for (i, step) in steps.iter().enumerate() {
            if !step.select.is_empty() {
                if !step.actions.is_empty() {
                    return Err((
                        Some(i),
                        "a branching step fires no actions — put them in the step \
                         before, or inside the arms"
                            .to_string(),
                    ));
                }
                if !matches!(step.transition, Condition::Immediately) {
                    return Err((
                        Some(i),
                        "a branching step's arms are its transitions; leave its own \
                         transition empty (immediately)"
                            .to_string(),
                    ));
                }
                let entry_tracked = tracked.clone();
                let entry_held = held.clone();
                let mut rejoin: Option<(Vec<_>, std::collections::HashMap<_, _>)> = None;
                for (j, arm) in step.select.iter().enumerate() {
                    let wrap =
                        |m: String| (Some(i), format!("arm {} of `{}`: {m}", j + 1, step.name));
                    if arm.condition.mentions_done() {
                        return Err(wrap(
                            "`done` waits for a motion/ramp, but a branching step \
                             starts none — await it in the step before"
                                .to_string(),
                        ));
                    }
                    self.validate_condition(&arm.condition).map_err(wrap)?;
                    let mut arm_tracked = entry_tracked.clone();
                    let mut arm_held = entry_held.clone();
                    self.validate_steps(&arm.steps, &mut arm_tracked, &mut arm_held)
                        .map_err(|(_, m)| wrap(m))?;
                    match &rejoin {
                        None => rejoin = Some((arm_tracked, arm_held)),
                        Some((t, h)) if *t == arm_tracked && *h == arm_held => {}
                        Some(_) => {
                            return Err((
                                Some(i),
                                format!(
                                    "the arms of `{}` rejoin with different grasp/tracking \
                                     state — end every arm the same way (release or hand \
                                     over before rejoining)",
                                    step.name
                                ),
                            ))
                        }
                    }
                }
                let (t, h) = rejoin.expect("select verified non-empty");
                *tracked = t;
                *held = h;
                continue;
            }
            for action in &step.actions {
                self.validate_action(action, held)
                    .map_err(|m| (Some(i), m))?;
                self.validate_tracking(action, tracked)
                    .map_err(|m| (Some(i), m))?;
            }
            // One driver per joint per step: two moves on one arm fight for
            // the same joints; one move per arm — of one robot or of two —
            // is the multi-actor case.
            let mut driving: Vec<(usize, Vec<usize>)> = Vec::new();
            for action in &step.actions {
                if let Some((robot, joints)) =
                    self.driver_joints(action).map_err(|m| (Some(i), m))?
                {
                    if let Some((_, other)) = driving
                        .iter()
                        .find(|(r, other)| *r == robot && other.iter().any(|j| joints.contains(j)))
                    {
                        let model = &self.robots()[robot].model;
                        let shared = joints
                            .iter()
                            .find(|j| other.contains(j))
                            .map(|&qi| model.joints[model.actuated_joints[qi]].name.clone())
                            .unwrap_or_default();
                        return Err((
                            Some(i),
                            format!(
                                "a step can start at most one motion or ramp per robot arm \
                                 (they drive the same joints); `{shared}` of `{}` already has one",
                                self.robots()[robot].name
                            ),
                        ));
                    }
                    driving.push((robot, joints));
                }
            }
            if step.transition.mentions_done() && driving.is_empty() {
                return Err((
                    Some(i),
                    "`done` waits for a motion/ramp, but this step starts none".to_string(),
                ));
            }
            self.validate_condition(&step.transition)
                .map_err(|m| (Some(i), m))?;
        }
        Ok(())
    }

    /// Threads the per-arm tracking mode through one action, rejecting the
    /// orders the scan engine cannot honour. `tracked[r]` carries every
    /// track robot `r` holds: the arm following, the joints it spends, the
    /// object, and the joints that would fight the track if a ramp drove
    /// them.
    fn validate_tracking(
        &self,
        action: &Action,
        tracked: &mut [Vec<TrackClaim>],
    ) -> Result<(), String> {
        match action {
            Action::Track {
                robot,
                object,
                link,
                group,
            } => {
                let r = self.resolve_seq_robot(robot)?;
                let model = &self.robots()[r].model;
                let link_index = match link {
                    Some(name) => Some(
                        model
                            .link_index(name)
                            .ok_or_else(|| format!("unknown link `{name}`"))?,
                    ),
                    None => None,
                };
                let g = self
                    .resolve_group(r, group.as_deref(), link_index)
                    .map_err(|e| e.to_string())?;
                let joints = self.group_joints(r, g);
                if let Some(claim) = tracked[r]
                    .iter()
                    .find(|c| c.joints.iter().any(|j| joints.contains(j)))
                {
                    return Err(format!(
                        "already tracking `{}`; release it with untrack before tracking `{object}`",
                        claim.object
                    ));
                }
                // The wrist, not the fingertip: joints below the arm's tool
                // mount (a gripper's) move the servoed link without being
                // described by its pose, so the solver would trade the grip
                // away for reach.
                let mount = self.group_tool_mount(r, g);
                let servoed = link_index.unwrap_or(mount);
                let trunk = model.driving_joints(mount);
                let contested = model
                    .driving_joints(servoed)
                    .into_iter()
                    .filter(|ji| !trunk.contains(ji))
                    .map(|ji| model.joints[ji].name.clone())
                    .collect();
                tracked[r].push(TrackClaim {
                    group: g,
                    joints,
                    object: object.clone(),
                    contested,
                    mount: model.links[mount].name.clone(),
                });
                Ok(())
            }
            Action::Untrack { robot, group } => {
                let r = self.resolve_seq_robot(robot)?;
                let claims = &mut tracked[r];
                let index = match group {
                    Some(name) => {
                        let g = self.robots()[r]
                            .model
                            .group_index(name)
                            .ok_or_else(|| format!("unknown group `{name}`"))?;
                        claims
                            .iter()
                            .position(|c| c.group == Some(g))
                            .ok_or_else(|| format!("`{name}` has no active track"))?
                    }
                    None => match claims.len() {
                        0 => return Err("untrack without an active track".to_string()),
                        1 => 0,
                        _ => {
                            return Err(format!(
                                "`{}` tracks with several arms ({}); name the group to release",
                                self.robots()[r].name,
                                claims
                                    .iter()
                                    .map(|c| self.group_label(r, c.group))
                                    .collect::<Vec<_>>()
                                    .join(", ")
                            ))
                        }
                    },
                };
                claims.remove(index);
                Ok(())
            }
            Action::StartRamp { robot, targets, .. } => {
                let r = self.resolve_seq_robot(robot)?;
                for claim in &tracked[r] {
                    if let Some((joint, _)) = targets
                        .iter()
                        .find(|(joint, _)| claim.contested.iter().any(|c| c == joint))
                    {
                        return Err(format!(
                            "ramping `{joint}` while tracking `{}` fights the track: it moves the \
                             servoed link itself, so the solve would spend it chasing the part. Track a \
                             link at or above the tool mount (`{}`) instead",
                            claim.object, claim.mount
                        ));
                    }
                }
                Ok(())
            }
            // A policy commands its joints outright: it cannot ride a
            // track's offset any more than a ramp can.
            Action::Policy { policy, robot, .. } => {
                let r = self.resolve_seq_robot(robot)?;
                if let Some(claim) = tracked[r].first() {
                    return Err(format!(
                        "policy `{policy}` cannot drive `{}` while it tracks `{}`; untrack first",
                        self.robots()[r].name,
                        claim.object
                    ));
                }
                Ok(())
            }
            // Planned motions bake their whole trajectory when they start,
            // which cannot absorb a part that keeps moving underneath. Only
            // the tracking arm's own joints conflict — the other arm may
            // plan.
            Action::StartMotion { motion } => {
                let Some(found) = self.motions.iter().find(|m| &m.name == motion) else {
                    return Ok(());
                };
                let joints = self.motion_joints(found);
                match tracked[found.robot]
                    .iter()
                    .find(|c| c.joints.iter().any(|j| joints.contains(j)))
                {
                    Some(claim) => Err(format!(
                        "motion `{motion}` cannot run while tracking `{}`: \
                         plans are baked in world coordinates, so release the track first",
                        claim.object
                    )),
                    None => Ok(()),
                }
            }
            _ => Ok(()),
        }
    }

    /// The joints an arm spends: the group's, or every joint when the
    /// whole robot is meant (`None`).
    pub(crate) fn group_joints(&self, robot: usize, group: Option<usize>) -> Vec<usize> {
        let model = &self.robots()[robot].model;
        match group.and_then(|g| model.groups().into_iter().nth(g)) {
            Some(g) => g.joints,
            None => (0..model.dof()).collect(),
        }
    }

    /// The tool mount an arm's track servos by default: the group's, or
    /// the robot's.
    pub(crate) fn group_tool_mount(&self, robot: usize, group: Option<usize>) -> usize {
        let model = &self.robots()[robot].model;
        match group.and_then(|g| model.groups().into_iter().nth(g)) {
            Some(g) => model.group_tool_mount(&g),
            None => model.tool_mount_link(),
        }
    }

    /// A group's name for messages (`*` for the whole robot).
    pub(crate) fn group_label(&self, robot: usize, group: Option<usize>) -> String {
        group
            .and_then(|g| self.robots()[robot].model.groups().into_iter().nth(g))
            .map(|g| g.name)
            .unwrap_or_else(|| "*".to_string())
    }

    /// A vehicle definition the scan engine can honour: a walkable path,
    /// resolvable stations and body obstacles, positive rates. Run once per
    /// simulate, so a broken definition fails with a message instead of a
    /// runtime surprise.
    fn validate_vehicle(&self, device: &Device) -> Result<(), String> {
        let DeviceKind::Vehicle {
            path,
            wheels,
            body,
            speed,
            turn_speed,
            start,
            tray,
            drive,
            members: _,
        } = &device.kind
        else {
            return Ok(());
        };
        for wheel in wheels {
            wheel.validate(self, body)?;
        }
        if let Some((_, size)) = tray {
            if size.iter().any(|v| !(v.is_finite() && *v > 0.0)) {
                return Err(format!(
                    "vehicle `{}` tray size must be positive, got {size:?}",
                    device.name
                ));
            }
        }
        let dev = &device.name;
        if path.waypoints.len() < 2 {
            return Err(format!(
                "vehicle `{dev}` needs at least 2 waypoints, got {}",
                path.waypoints.len()
            ));
        }
        if path.stations.is_empty() {
            return Err(format!("vehicle `{dev}` has no stations"));
        }
        for (name, index) in &path.stations {
            if *index >= path.waypoints.len() {
                return Err(format!(
                    "vehicle `{dev}` station `{name}` points at waypoint {index}, \
                     but the path has {}",
                    path.waypoints.len()
                ));
            }
            if path
                .stations
                .iter()
                .filter(|(other, _)| other == name)
                .count()
                > 1
            {
                return Err(format!("vehicle `{dev}` has two stations named `{name}`"));
            }
        }
        if path.station(start).is_none() {
            let known: Vec<String> = path
                .stations
                .iter()
                .map(|(n, _)| format!("`{n}`"))
                .collect();
            return Err(format!(
                "vehicle `{dev}` starts at unknown station `{start}` (stations: {})",
                known.join(", ")
            ));
        }
        if !(speed.is_finite() && *speed > 0.0) {
            return Err(format!(
                "vehicle `{dev}` speed must be positive, got {speed}"
            ));
        }
        if !(turn_speed.is_finite() && *turn_speed > 0.0) {
            return Err(format!(
                "vehicle `{dev}` turn_speed must be positive, got {turn_speed}"
            ));
        }
        if let Some(v) = drive.reverse_speed_stated() {
            if !(v.is_finite() && v > 0.0) {
                return Err(format!(
                    "vehicle `{dev}` reverse_speed must be positive, got {v}"
                ));
            }
        }
        if let Drive::Steered { turn_radius, .. } = drive {
            if !(turn_radius.is_finite() && *turn_radius > 0.0) {
                return Err(format!(
                    "vehicle `{dev}` turn_radius must be positive, got {turn_radius}"
                ));
            }
            // A steered drive rounds corners on the level, and stops only
            // where it can leave along the line it arrived by.
            let n = path.waypoints.len();
            if let Some(w) = path
                .waypoints
                .iter()
                .find(|w| (w.z - path.waypoints[0].z).abs() > 1e-9)
            {
                return Err(format!(
                    "vehicle `{dev}` has a turn_radius, which drives level paths only, but its \
                     path climbs to z = {:.3} m",
                    w.z
                ));
            }
            for (name, at) in &path.stations {
                let at = *at;
                let prev = if at > 0 {
                    Some(at - 1)
                } else if path.ring && n > 2 {
                    Some(n - 1)
                } else {
                    None
                };
                let next = if at + 1 < n {
                    Some(at + 1)
                } else if path.ring && n > 2 {
                    Some(0)
                } else {
                    None
                };
                let (Some(i), Some(j)) = (prev, next) else {
                    continue;
                };
                let a = path.waypoints[at] - path.waypoints[i];
                let b = path.waypoints[j] - path.waypoints[at];
                if a.norm() < 1e-9 || b.norm() < 1e-9 {
                    continue;
                }
                let cross = (a.x * b.y - a.y * b.x).abs() / (a.norm() * b.norm());
                if cross > 1e-6 {
                    return Err(format!(
                        "vehicle `{dev}` station `{name}` (waypoint {at}) sits on a {:.1}° corner, \
                         but a steered drive arrives at and leaves a station along one line — \
                         put the corner on the waypoint before the station",
                        cross.asin().to_degrees()
                    ));
                }
            }
        }
        // Gears are a differential drive's: a holonomic machine never
        // turns to face a leg, an aerial one flies its yaw policy.
        for (i, (name, gear)) in path.arrivals.iter().enumerate() {
            if path.station(name).is_none() {
                return Err(format!(
                    "vehicle `{dev}` states an arrival gear for `{name}`, which is not a station"
                ));
            }
            if path.arrivals[..i].iter().any(|(n, _)| n == name) {
                return Err(format!(
                    "vehicle `{dev}` states two arrival gears for station `{name}`"
                ));
            }
            if !drive.has_gears() {
                return Err(format!(
                    "vehicle `{dev}` station `{name}` arrives in {gear:?} gear, but only a \
                     differential or steered drive has gears — a holonomic or aerial machine \
                     docks facing whatever it faced when parked"
                ));
            }
        }
        if let Drive::Aerial {
            climb_speed,
            descent_speed,
            ..
        } = drive
        {
            // The air has no grade and no lift edges: z is the machine's
            // own axis. Only the rates have to be real.
            if !(climb_speed.is_finite() && *climb_speed > 0.0) {
                return Err(format!(
                    "vehicle `{dev}` climb_speed must be positive, got {climb_speed}"
                ));
            }
            if !(descent_speed.is_finite() && *descent_speed > 0.0) {
                return Err(format!(
                    "vehicle `{dev}` descent_speed must be positive, got {descent_speed}"
                ));
            }
            for name in body {
                if !self.obstacles.iter().any(|o| &o.name == name) {
                    return Err(format!(
                        "vehicle `{dev}` body names unknown obstacle `{name}`"
                    ));
                }
            }
            return Ok(());
        }
        if let Some(limit) = drive.max_grade() {
            if !(limit.is_finite() && limit > 0.0) {
                return Err(format!(
                    "vehicle `{dev}` max_grade must be positive (rise over run), got {limit}"
                ));
            }
        }
        // The z profile a ground drive can honour: level legs always,
        // graded legs only within a declared ability, vertical stacks
        // never — climbing straight up is a lift's job, not a wheel's.
        let n = path.waypoints.len();
        let edges = (0..n.saturating_sub(1))
            .map(|i| (i, i + 1))
            .chain((path.ring && n > 1).then_some((n - 1, 0)));
        for (i, j) in edges {
            let d = path.waypoints[j] - path.waypoints[i];
            if d.norm() < 1e-9 {
                continue;
            }
            let run = d.x.hypot(d.y);
            let rise = d.z.abs();
            if rise <= 1e-9 {
                continue;
            }
            if run <= 1e-9 {
                // A vertical edge is never driven: it is ridden. Legal
                // only when a lift's capture zone covers both ends at its
                // stops — the hop the route refuses to walk (§goto) and
                // the ride performs.
                if self.lift_covers(&path.waypoints[i], &path.waypoints[j]) {
                    continue;
                }
                return Err(format!(
                    "vehicle `{dev}` path is vertical between waypoints {i} and {j} \
                     (Δz = {:.3} m with no horizontal run) — a ground drive cannot \
                     climb straight up; that hop is a lift's job (put both ends \
                     inside a lift's capture zone at its stops)",
                    d.z
                ));
            }
            let grade = rise / run;
            match drive.max_grade() {
                None => {
                    return Err(format!(
                        "vehicle `{dev}` path climbs {:.1}° between waypoints {i} \
                         and {j}, but the drive declares no max_grade — pass \
                         max_grade (rise over run, e.g. 0.10 for 10 %) to allow \
                         slopes",
                        grade.atan().to_degrees()
                    ))
                }
                Some(limit) if grade > limit + 1e-12 => {
                    return Err(format!(
                        "vehicle `{dev}` path climbs {:.1}° ({:.1} %) between \
                         waypoints {i} and {j}, over the drive's max_grade {:.1} %",
                        grade.atan().to_degrees(),
                        grade * 100.0,
                        limit * 100.0
                    ))
                }
                _ => {}
            }
        }
        for name in body {
            if !self.obstacles.iter().any(|o| &o.name == name) {
                return Err(format!(
                    "vehicle `{dev}` body names unknown obstacle `{name}`"
                ));
            }
        }
        Ok(())
    }

    /// Does some lift's capture zone contain both `a` and `b`, each at
    /// one of its stops? The legality test for a vertical path edge: the
    /// hop is ridden, not driven.
    fn lift_covers(&self, a: &nalgebra::Point3<f64>, b: &nalgebra::Point3<f64>) -> bool {
        self.devices.iter().any(|d| {
            let DeviceKind::Lift {
                zone_pose,
                zone_size,
                axis,
                stops,
                ..
            } = &d.kind
            else {
                return false;
            };
            let half = zone_size / 2.0;
            let at_a_stop = |p: &nalgebra::Point3<f64>| {
                stops.iter().any(|(_, v)| {
                    let local = zone_pose.inverse_transform_point(&(p - axis.into_inner() * *v));
                    local.x.abs() <= half.x && local.y.abs() <= half.y && local.z.abs() <= half.z
                })
            };
            at_a_stop(a) && at_a_stop(b)
        })
    }

    /// A coil-driven axis the exchange can carry: every coil names one of
    /// the stops, no two coils share a name, and at most one stop is left
    /// without a coil (the spring-return position).
    fn validate_axis(&self, device: &Device) -> Result<(), String> {
        let DeviceKind::LinearAxis { stops, coils, .. } = &device.kind else {
            return Ok(());
        };
        let dev = &device.name;
        for (coil, stop) in coils {
            if coil.is_empty() || coil.contains('/') {
                return Err(format!(
                    "axis `{dev}` coil name {coil:?} must be non-empty and contain no `/`"
                ));
            }
            if coils.iter().filter(|(c, _)| c == coil).count() > 1 {
                return Err(format!("axis `{dev}` has two coils named `{coil}`"));
            }
            if !stops.iter().any(|(n, _)| n == stop) {
                let known: Vec<String> = stops.iter().map(|(n, _)| format!("`{n}`")).collect();
                return Err(format!(
                    "axis `{dev}` coil `{coil}` drives to unknown stop `{stop}` (stops: {})",
                    known.join(", ")
                ));
            }
        }
        if !coils.is_empty() {
            let uncoiled = stops
                .iter()
                .filter(|(n, _)| !coils.iter().any(|(_, s)| s == n))
                .count();
            if uncoiled > 1 {
                return Err(format!(
                    "axis `{dev}`: {uncoiled} stops have no coil — a cylinder has one spring-return stop at most"
                ));
            }
        }
        Ok(())
    }

    /// A lift definition the scan engine can honour: a real car, resolvable
    /// stops, positive rates.
    fn validate_lift(&self, device: &Device) -> Result<(), String> {
        let DeviceKind::Lift {
            car,
            zone_size,
            speed,
            stops,
            start,
            ..
        } = &device.kind
        else {
            return Ok(());
        };
        let dev = &device.name;
        if zone_size.iter().any(|v| !(v.is_finite() && *v > 0.0)) {
            return Err(format!(
                "lift `{dev}` zone size must be positive, got {zone_size:?}"
            ));
        }
        if !(speed.is_finite() && *speed > 0.0) {
            return Err(format!("lift `{dev}` speed must be positive, got {speed}"));
        }
        if stops.is_empty() {
            return Err(format!("lift `{dev}` has no stops"));
        }
        for (name, value) in stops {
            if !value.is_finite() {
                return Err(format!("lift `{dev}` stop `{name}` is not finite"));
            }
            if stops.iter().filter(|(other, _)| other == name).count() > 1 {
                return Err(format!("lift `{dev}` has two stops named `{name}`"));
            }
        }
        if !stops.iter().any(|(n, _)| n == start) {
            let known: Vec<String> = stops.iter().map(|(n, _)| format!("`{n}`")).collect();
            return Err(format!(
                "lift `{dev}` starts at unknown stop `{start}` (stops: {})",
                known.join(", ")
            ));
        }
        for name in car {
            if !self.obstacles.iter().any(|o| &o.name == name) {
                return Err(format!("lift `{dev}` car names unknown obstacle `{name}`"));
            }
        }
        Ok(())
    }

    fn validate_action(
        &self,
        action: &Action,
        held: &mut std::collections::HashMap<String, (usize, usize)>,
    ) -> Result<(), String> {
        match action {
            Action::StartMotion { motion } => {
                let found = self
                    .motions
                    .iter()
                    .find(|m| &m.name == motion)
                    .ok_or_else(|| format!("unknown motion `{motion}`"))?;
                if found.segments.is_empty() {
                    return Err(format!("motion `{motion}` has no segments"));
                }
                Ok(())
            }
            Action::StartToolpath { robot, toolpath } => {
                self.resolve_seq_robot(robot)?;
                let found = self
                    .toolpaths()
                    .iter()
                    .find(|t| &t.name == toolpath)
                    .ok_or_else(|| format!("unknown toolpath `{toolpath}`"))?;
                if found.target_count() == 0 {
                    return Err(format!("toolpath `{toolpath}` has no targets"));
                }
                Ok(())
            }
            Action::Policy {
                policy,
                robot,
                group,
                hz,
                max_duration,
            } => {
                if policy.is_empty() {
                    return Err("policy step names no policy".to_string());
                }
                let r = self.resolve_seq_robot(robot)?;
                if let Some(name) = group {
                    self.robots()[r].model.group_index(name).ok_or_else(|| {
                        format!("unknown group `{name}` on `{}`", self.robots()[r].name)
                    })?;
                }
                if !(hz.is_finite() && *hz > 0.0) {
                    return Err(format!("policy rate must be positive, got {hz} Hz"));
                }
                if !(max_duration.is_finite() && *max_duration > 0.0) {
                    return Err(format!(
                        "policy max_duration must be positive, got {max_duration}"
                    ));
                }
                Ok(())
            }
            Action::StartRamp {
                robot,
                targets,
                duration,
                ..
            } => {
                let model = &self.robots()[self.resolve_seq_robot(robot)?].model;
                if targets.is_empty() {
                    return Err("ramp has no target joints".to_string());
                }
                if !(duration.is_finite() && *duration > 0.0) {
                    return Err(format!("ramp duration must be positive, got {duration}"));
                }
                for (joint, value) in targets {
                    let ji = model
                        .joint_index(joint)
                        .ok_or_else(|| format!("unknown joint `{joint}`"))?;
                    let j = &model.joints[ji];
                    if j.q_index.is_none() {
                        // A mimic joint is commandable, just not directly:
                        // say which joint to ramp instead.
                        return Err(match j.mimic {
                            Some(m) => format!(
                                "joint `{joint}` follows `{}`; ramp that joint instead",
                                model.joints[m.source_joint].name
                            ),
                            None => format!("joint `{joint}` is not actuated"),
                        });
                    }
                    if let Some(l) = &j.limits {
                        if *value < l.lower - 1e-9 || *value > l.upper + 1e-9 {
                            return Err(format!(
                                "ramp target {value} for `{joint}` is outside [{}, {}]",
                                l.lower, l.upper
                            ));
                        }
                    }
                }
                Ok(())
            }
            Action::Attach {
                robot,
                object,
                link,
                group,
                ..
            } => {
                let r = self.resolve_seq_robot(robot)?;
                if !self.obstacles.iter().any(|o| &o.name == object) {
                    return Err(format!("unknown obstacle `{object}`"));
                }
                let model = &self.robots()[r].model;
                let link_index = match link {
                    Some(link) => Some(
                        model
                            .link_index(link)
                            .ok_or_else(|| format!("unknown link `{link}`"))?,
                    ),
                    None => None,
                };
                let g = self
                    .resolve_group(r, group.as_deref(), link_index)
                    .map_err(|e| e.to_string())?;
                let anchor = link_index.unwrap_or_else(|| self.group_tip(r, g));
                // A grasp bound to a coil is the plant's effect, not a
                // step's command: the two would attach twice and release
                // out of step.
                if let Some(b) = self.grasp_binding_at(r, anchor) {
                    return Err(format!(
                        "`{}` grasps by its coil `{}` (bind_grasp): set the signal and the plant \
                         takes the part — drop the attach",
                        self.robots()[r].name,
                        b.signal
                    ));
                }
                // One carrier at a time; a handover is written as
                // "detach → (place) → attach" (§design-multi-robot 6).
                if let Some((carrier, _)) = held.get(object) {
                    return Err(format!(
                        "`{object}` is already attached to `{}`; detach it first \
                         (a handover is detach → attach)",
                        self.robots()[*carrier].name
                    ));
                }
                held.insert(object.clone(), (r, anchor));
                Ok(())
            }
            Action::Detach { object } => {
                if !self.obstacles.iter().any(|o| &o.name == object) {
                    return Err(format!("unknown obstacle `{object}`"));
                }
                if held.remove(object).is_none() {
                    return Err(format!(
                        "`{object}` is not attached at this point in the sequence"
                    ));
                }
                Ok(())
            }
            Action::Track {
                robot,
                object,
                link,
                group,
            } => {
                let r = self.resolve_seq_robot(robot)?;
                if !self.obstacles.iter().any(|o| &o.name == object) {
                    return Err(format!("unknown obstacle `{object}`"));
                }
                let model = &self.robots()[r].model;
                let link_index = match link {
                    Some(link) => Some(
                        model
                            .link_index(link)
                            .ok_or_else(|| format!("unknown link `{link}`"))?,
                    ),
                    None => None,
                };
                let g = self
                    .resolve_group(r, group.as_deref(), link_index)
                    .map_err(|e| e.to_string())?;
                // A part in this arm's own hand would be chased in a
                // circle. One held by another arm — of this robot or
                // another — moves, and following it is the point (a
                // two-handed hold).
                if let Some(&(carrier, anchor)) = held.get(object) {
                    let joints = self.group_joints(r, g);
                    let own = carrier == r
                        && model.driving_joints(anchor).iter().any(|ji| {
                            model.joints[*ji]
                                .q_index
                                .is_some_and(|qi| joints.contains(&qi))
                        });
                    if own {
                        return Err(format!(
                            "`{object}` is already grasped by this arm; there is nothing to track"
                        ));
                    }
                }
                Ok(())
            }
            Action::Untrack { robot, .. } => self.resolve_seq_robot(robot).map(|_| ()),
            Action::Crouch {
                robot,
                depth,
                lean,
                duration,
            } => {
                let r = self.resolve_seq_robot(robot)?;
                self.crouch_legs(r)?;
                if !(depth.is_finite() && *depth >= 0.0) {
                    return Err(format!(
                        "crouch depth is how far the body goes down, in metres (>= 0), got {depth}"
                    ));
                }
                if !lean.is_finite() {
                    return Err(format!("crouch lean must be finite, got {lean}"));
                }
                if let Some(d) = duration {
                    if !(d.is_finite() && *d > 0.0) {
                        return Err(format!("crouch duration must be positive, got {d}"));
                    }
                }
                Ok(())
            }
            Action::SetWord { word, value } => self.validate_word_write(word, value),
            Action::Set { signal, .. } => {
                if !self.signals.iter().any(|s| &s.name == signal) {
                    if self.sensors.iter().any(|s| &s.name == signal) {
                        return Err(format!("sensor `{signal}` is a read-only input"));
                    }
                    return Err(format!(
                        "unknown signal `{signal}` (declare it with define_signal)"
                    ));
                }
                Ok(())
            }
            Action::Device { device, command } => {
                let found = self
                    .devices
                    .iter()
                    .find(|d| &d.name == device)
                    .ok_or_else(|| format!("unknown device `{device}`"))?;
                match (&found.kind, command) {
                    (DeviceKind::Vehicle { path, .. }, DeviceCommand::Goto { station }) => {
                        if path.station(station).is_none() {
                            let known: Vec<String> = path
                                .stations
                                .iter()
                                .map(|(n, _)| format!("`{n}`"))
                                .collect();
                            return Err(format!(
                                "vehicle `{device}` has no station `{station}` (stations: {})",
                                known.join(", ")
                            ));
                        }
                        Ok(())
                    }
                    (DeviceKind::Vehicle { .. }, _) => {
                        Err(format!("vehicle `{device}` only takes goto commands"))
                    }
                    (_, DeviceCommand::Goto { .. }) => Err(format!(
                        "`{device}` is not a vehicle; goto drives a vehicle to a station"
                    )),
                    (DeviceKind::Conveyor { .. }, DeviceCommand::Start | DeviceCommand::Stop) => {
                        Ok(())
                    }
                    (DeviceKind::Conveyor { .. }, DeviceCommand::SetSpeed(v)) => {
                        if v.is_finite() {
                            Ok(())
                        } else {
                            Err(format!("set_speed({v}) is not finite"))
                        }
                    }
                    (DeviceKind::Conveyor { .. }, DeviceCommand::MoveTo(_)) => Err(format!(
                        "conveyor `{device}` has no position; use start/stop"
                    )),
                    (DeviceKind::Conveyor { velocity, .. }, DeviceCommand::Advance(d)) => {
                        if !d.is_finite() || *d < 0.0 {
                            Err(format!("advance({d}) is not a distance"))
                        } else if velocity.norm() < 1e-12 {
                            Err(format!(
                                "conveyor `{device}` has zero speed; a fixed advance \
                                 needs a velocity to run at"
                            ))
                        } else {
                            Ok(())
                        }
                    }
                    (_, DeviceCommand::Advance(_)) => Err(format!(
                        "`{device}` is not a conveyor; advance is the indexed-transfer command"
                    )),
                    (DeviceKind::Source { .. }, DeviceCommand::Start | DeviceCommand::Stop) => {
                        Ok(())
                    }
                    (DeviceKind::Source { .. }, DeviceCommand::SetSpeed(v)) => {
                        if v.is_finite() && *v >= 0.0 {
                            Ok(())
                        } else {
                            Err(format!("set_speed({v}) is not a feed period"))
                        }
                    }
                    (DeviceKind::Source { .. }, DeviceCommand::MoveTo(_)) => {
                        Err(format!("source `{device}` has no position; use start/stop"))
                    }
                    (DeviceKind::Sink { .. }, _) => Err(format!(
                        "sink `{device}` is always collecting and takes no command"
                    )),
                    (DeviceKind::LinearAxis { range, .. }, DeviceCommand::MoveTo(p)) => {
                        if !p.is_finite() || *p < range.0 - 1e-9 || *p > range.1 + 1e-9 {
                            Err(format!(
                                "move_to({p}) is outside the axis range [{}, {}]",
                                range.0, range.1
                            ))
                        } else {
                            Ok(())
                        }
                    }
                    (DeviceKind::LinearAxis { stops, .. }, DeviceCommand::MoveToStop(stop)) => {
                        if stops.iter().any(|(n, _)| n == stop) {
                            Ok(())
                        } else {
                            let known: Vec<String> =
                                stops.iter().map(|(n, _)| format!("`{n}`")).collect();
                            Err(format!(
                                "axis `{device}` has no stop `{stop}` (stops: {})",
                                if known.is_empty() { "none".to_string() } else { known.join(", ") }
                            ))
                        }
                    }
                    (DeviceKind::LinearAxis { .. }, _) => Err(format!(
                        "axis `{device}` only takes move_to commands (a position, or one of its stops)"
                    )),
                    (DeviceKind::Lift { stops, .. }, DeviceCommand::MoveToStop(stop)) => {
                        if stops.iter().any(|(n, _)| n == stop) {
                            Ok(())
                        } else {
                            let known: Vec<String> =
                                stops.iter().map(|(n, _)| format!("`{n}`")).collect();
                            Err(format!(
                                "lift `{device}` has no stop `{stop}` (stops: {})",
                                known.join(", ")
                            ))
                        }
                    }
                    (DeviceKind::Lift { .. }, _) => Err(format!(
                        "lift `{device}` moves to named stops; use \
                         bt.seq.move_to({device:?}, \"2F\")"
                    )),
                    (_, DeviceCommand::MoveToStop(_)) => Err(format!(
                        "`{device}` is not a lift or an axis with named stops; move_to with a \
                         stop name drives one of those"
                    )),
                }
            }
        }
    }

    fn validate_condition(&self, condition: &Condition) -> Result<(), String> {
        match condition {
            Condition::Immediately | Condition::Done => Ok(()),
            Condition::RobotDone { robot } => {
                self.resolve_seq_robot(&Some(robot.clone())).map(|_| ())
            }
            Condition::GroupDone { robot, group } => {
                let r = self.resolve_seq_robot(&Some(robot.clone()))?;
                self.robots()[r]
                    .model
                    .group_index(group)
                    .map(|_| ())
                    .ok_or_else(|| format!("unknown group `{group}` on `{robot}`"))
            }
            Condition::Elapsed { seconds } => {
                if !(seconds.is_finite() && *seconds >= 0.0) {
                    return Err(format!("elapsed seconds must be >= 0, got {seconds}"));
                }
                Ok(())
            }
            Condition::Signal { name, .. }
            | Condition::Rising { name }
            | Condition::Falling { name } => {
                if !self.signal_readable(name) {
                    return Err(format!(
                        "unknown signal `{name}` (declare it with define_signal or add a sensor)"
                    ));
                }
                Ok(())
            }
            Condition::DeviceDone { device } => {
                let found = self
                    .devices
                    .iter()
                    .find(|d| &d.name == device)
                    .ok_or_else(|| format!("unknown device `{device}`"))?;
                if !matches!(
                    found.kind,
                    DeviceKind::LinearAxis { .. }
                        | DeviceKind::Vehicle { .. }
                        | DeviceKind::Lift { .. }
                        // A conveyor's "in-position" is a fixed advance
                        // consumed — the await for `Advance`.
                        | DeviceKind::Conveyor { .. }
                ) {
                    return Err(format!(
                        "`device_done` waits for a positioning device \
                         (linear axis, vehicle, or an advancing conveyor); \
                         `{device}` has no in-position"
                    ));
                }
                Ok(())
            }
            Condition::All(cs) | Condition::Any(cs) => {
                if cs.is_empty() {
                    return Err("empty condition group".to_string());
                }
                cs.iter().try_for_each(|c| self.validate_condition(c))
            }
            Condition::Not(c) => self.validate_condition(c),
            Condition::Compare { left, right, .. } => {
                self.validate_operand(left)?;
                self.validate_operand(right)
            }
            Condition::StepActive { sequence, step } => {
                let program = self
                    .sequence(sequence)
                    .ok_or_else(|| format!("step_active: unknown sequence `{sequence}`"))?;
                fn has(steps: &[Step], name: &str) -> bool {
                    steps
                        .iter()
                        .any(|s| s.name == name || s.select.iter().any(|a| has(&a.steps, name)))
                }
                if !has(&program.steps, step) {
                    return Err(format!(
                        "step_active: sequence `{sequence}` has no step `{step}`"
                    ));
                }
                Ok(())
            }
        }
    }
}
