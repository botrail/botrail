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
    All,
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
    Conveyor {
        zone_pose: Isometry3<f64>,
        zone_size: Vector3<f64>,
        velocity: Vector3<f64>,
        running: bool,
    },
    /// Moves the listed obstacles along `axis` at `speed`, positioned by
    /// `MoveTo` commands within `range`; `DeviceDone` means in-position.
    LinearAxis {
        objects: Vec<String>,
        axis: Unit<Vector3<f64>>,
        speed: f64,
        position: f64,
        range: (f64, f64),
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

impl Drive {
    pub fn allow_reverse(&self) -> bool {
        match self {
            Drive::Differential { allow_reverse, .. } => *allow_reverse,
            Drive::Holonomic { .. } | Drive::Aerial { .. } => false,
        }
    }

    pub fn max_grade(&self) -> Option<f64> {
        match self {
            Drive::Differential { max_grade, .. } | Drive::Holonomic { max_grade } => *max_grade,
            Drive::Aerial { .. } => None,
        }
    }
}

impl Default for Drive {
    fn default() -> Self {
        Drive::Differential {
            allow_reverse: false,
            max_grade: None,
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
}

impl VehiclePath {
    /// Waypoint index of the named station.
    pub fn station(&self, name: &str) -> Option<usize> {
        self.stations
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, i)| *i)
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

/// The frame of a vehicle: its position on the guidance surface (z is the
/// floor height there) plus heading about +Z. The body stays level — pitch
/// and roll never enter, on a ramp the machine translates up it.
pub fn vehicle_frame(position: &nalgebra::Point3<f64>, heading: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        nalgebra::Translation3::new(position.x, position.y, position.z),
        nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), heading),
    )
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
    /// Set when the robot *is* the vehicle's legs: while the vehicle
    /// drives, the gait swings these legs so that every planted foot stays
    /// where it touched down. Not an action — a property of the mount, the
    /// way a wheel's spin is a property of the axle it sits on.
    pub gait: Option<GaitSpec>,
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
    /// `duration` seconds. Await it with [`Condition::Done`].
    StartRamp {
        robot: Option<String>,
        targets: Vec<(String, f64)>,
        duration: f64,
    },
    /// Grasp: rigidly attach an obstacle at its current relative pose
    /// (defaults as in [`Scene::attach_obstacle`]). Instantaneous.
    Attach {
        robot: Option<String>,
        object: String,
        link: Option<String>,
        touch_links: Option<Vec<String>>,
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
    },
    /// Release the track: commands go back to plain world coordinates from
    /// wherever the robot stands (never a jump). Instantaneous.
    Untrack { robot: Option<String> },
    /// Release: the obstacle's pose freezes where it is. Instantaneous.
    /// The carrying robot is looked up from the attachment.
    Detach { object: String },
    /// Write an internal signal (self-holding relay style).
    Set { signal: String, value: bool },
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
}

impl Action {
    /// The instance name this action is addressed to, when it carries one.
    /// `StartMotion` is absent on purpose: it names a motion, and the
    /// motion knows its own robot.
    pub(crate) fn robot_mut(&mut self) -> Option<&mut String> {
        match self {
            Action::StartRamp { robot, .. }
            | Action::Attach { robot, .. }
            | Action::Track { robot, .. }
            | Action::Untrack { robot }
            | Action::StartToolpath { robot, .. } => robot.as_mut(),
            _ => None,
        }
    }
}

impl Condition {
    fn mentions_done(&self) -> bool {
        match self {
            Condition::Done => true,
            Condition::All(cs) | Condition::Any(cs) => cs.iter().any(|c| c.mentions_done()),
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

    pub fn sensors(&self) -> &[Sensor] {
        &self.sensors
    }

    /// Adds or replaces a pseudo-sensor.
    pub fn upsert_sensor(&mut self, sensor: Sensor) {
        match self.sensors.iter_mut().find(|s| s.name == sensor.name) {
            Some(slot) => *slot = sensor,
            None => self.sensors.push(sensor),
        }
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
    fn resolve_faults(
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
        self.signals.iter().any(|s| s.name == name) || self.sensors.iter().any(|s| s.name == name)
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

    /// The robot a driver action moves: a motion's owner, or the ramp's
    /// addressed robot. `None` for non-driver actions.
    fn driver_robot(&self, action: &Action) -> Result<Option<usize>, String> {
        match action {
            Action::StartMotion { motion } => Ok(self
                .motions
                .iter()
                .find(|m| &m.name == motion)
                .map(|m| m.robot)),
            Action::StartRamp { robot, .. } | Action::StartToolpath { robot, .. } => {
                self.resolve_seq_robot(robot).map(Some)
            }
            _ => Ok(None),
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
        // resource key -> (kind label, display name, owning sequence index)
        let mut owners: HashMap<(u8, String), usize> = HashMap::new();
        let mut claim =
            |kind: u8, label: &str, name: String, program: usize| -> Result<(), String> {
                match owners.entry((kind, name.clone())) {
                    std::collections::hash_map::Entry::Vacant(v) => {
                        v.insert(program);
                        Ok(())
                    }
                    std::collections::hash_map::Entry::Occupied(o) if *o.get() == program => Ok(()),
                    std::collections::hash_map::Entry::Occupied(o) => Err(format!(
                        "{label} `{name}` is commanded by both `{}` and `{}`; every robot, \
                     device, and written signal belongs to one program (programs watch \
                     each other through conditions, not by sharing outputs)",
                        sequences[*o.get()].name,
                        sequences[program].name,
                    )),
                }
            };
        for (index, sequence) in sequences.iter().enumerate() {
            walk_actions(&sequence.steps, &mut |action| -> Result<(), String> {
                match action {
                    Action::StartMotion { motion } => {
                        // Validation has already established the motion
                        // exists; its robot is the claimed resource.
                        if let Some(robot) = self
                            .motions()
                            .iter()
                            .find(|m| &m.name == motion)
                            .map(|m| m.robot)
                        {
                            let name = self.robots()[robot].name.clone();
                            claim(0, "robot", name, index)?;
                        }
                    }
                    Action::StartRamp { robot, .. }
                    | Action::Attach { robot, .. }
                    | Action::Track { robot, .. }
                    | Action::Untrack { robot }
                    | Action::StartToolpath { robot, .. } => {
                        if let Ok(r) = self.resolve_seq_robot(robot) {
                            let name = self.robots()[r].name.clone();
                            claim(0, "robot", name, index)?;
                        }
                    }
                    // Detach names an object; the carrying robot is
                    // whoever attached it, which the same-owner rule
                    // for Attach already pins to one program.
                    Action::Detach { .. } => {}
                    Action::Set { signal, .. } => {
                        claim(1, "signal", signal.clone(), index)?;
                    }
                    Action::Device { device, .. } => {
                        claim(2, "device", device.clone(), index)?;
                    }
                }
                Ok(())
            })?;
        }
        Ok(())
    }

    /// Full authoring-time validation, run before a rollout. `Err` carries
    /// the offending step index (when applicable) and a message.
    pub(crate) fn validate_sequence(
        &self,
        sequence: &Sequence,
    ) -> Result<(), (Option<usize>, String)> {
        if sequence.steps.is_empty() {
            return Err((None, "sequence has no steps".to_string()));
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
            self.validate_lift(device).map_err(|m| (None, m))?;
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
            crate::gait::check_stride(&resolved, &mount.offset, speed, turn_speed)
                .map_err(|m| (None, format!("robot `{}`: {m}", robot.name)))?;
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
        let mut tracked: Vec<Option<(String, Vec<String>)>> = vec![None; self.robots().len()];
        let mut held: std::collections::HashMap<String, usize> = self
            .attachments()
            .iter()
            .map(|a| (a.object.clone(), a.robot))
            .collect();
        self.validate_steps(&sequence.steps, &mut tracked, &mut held)
    }

    /// The per-step walk of [`validate_sequence`], recursing into branch
    /// arms. Errors carry the local step index; branch recursion rewraps
    /// them onto the branching step with the arm named.
    fn validate_steps(
        &self,
        steps: &[Step],
        tracked: &mut Vec<Option<(String, Vec<String>)>>,
        held: &mut std::collections::HashMap<String, usize>,
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
                    self.validate_condition(&arm.condition).map_err(&wrap)?;
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
            // One driver per robot per step: two moves on one arm fight for
            // the same joints; one move per arm is the multi-actor case.
            let mut driving: Vec<usize> = Vec::new();
            for action in &step.actions {
                if let Some(robot) = self.driver_robot(action).map_err(|m| (Some(i), m))? {
                    if driving.contains(&robot) {
                        return Err((
                            Some(i),
                            format!(
                                "a step can start at most one motion or ramp per robot \
                                 (they drive the same joints); `{}` already has one",
                                self.robots()[robot].name
                            ),
                        ));
                    }
                    driving.push(robot);
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

    /// Threads the per-robot tracking mode through one action, rejecting
    /// the orders the scan engine cannot honour. `tracked[r]` carries the
    /// robot's tracked object plus the joints that would fight the track if
    /// a ramp drove them.
    fn validate_tracking(
        &self,
        action: &Action,
        tracked: &mut [Option<(String, Vec<String>)>],
    ) -> Result<(), String> {
        match action {
            Action::Track {
                robot,
                object,
                link,
            } => {
                let r = self.resolve_seq_robot(robot)?;
                let model = &self.robots()[r].model;
                match &tracked[r] {
                    Some((current, _)) => Err(format!(
                        "already tracking `{current}`; release it with untrack before tracking `{object}`"
                    )),
                    None => {
                        let servoed = match link {
                            Some(name) => model
                                .link_index(name)
                                .ok_or_else(|| format!("unknown link `{name}`"))?,
                            None => model.tool_mount_link(),
                        };
                        // Joints below the tool mount (a gripper's) move the
                        // servoed link without being described by its pose, so
                        // the solver would trade the grip away for reach.
                        let trunk = model.driving_joints(model.tool_mount_link());
                        let contested = model
                            .driving_joints(servoed)
                            .into_iter()
                            .filter(|ji| !trunk.contains(ji))
                            .map(|ji| model.joints[ji].name.clone())
                            .collect();
                        tracked[r] = Some((object.clone(), contested));
                        Ok(())
                    }
                }
            }
            Action::Untrack { robot } => {
                let r = self.resolve_seq_robot(robot)?;
                match tracked[r].take() {
                    Some(_) => Ok(()),
                    None => Err("untrack without an active track".to_string()),
                }
            }
            Action::StartRamp { robot, targets, .. } => {
                let r = self.resolve_seq_robot(robot)?;
                let model = &self.robots()[r].model;
                let Some((object, contested)) = &tracked[r] else {
                    return Ok(());
                };
                match targets
                    .iter()
                    .find(|(joint, _)| contested.iter().any(|c| c == joint))
                {
                    Some((joint, _)) => Err(format!(
                        "ramping `{joint}` while tracking `{object}` fights the track: it moves the \
                         servoed link itself, so the solve would spend it chasing the part. Track a \
                         link at or above the tool mount (`{}`) instead",
                        model.links[model.tool_mount_link()].name
                    )),
                    None => Ok(()),
                }
            }
            // Planned motions bake their whole trajectory when they start,
            // which cannot absorb a part that keeps moving underneath. Only
            // the owning robot's track conflicts — another arm may plan.
            Action::StartMotion { motion } => {
                let Some(owner) = self
                    .motions
                    .iter()
                    .find(|m| &m.name == motion)
                    .map(|m| m.robot)
                else {
                    return Ok(());
                };
                match &tracked[owner] {
                    Some((object, _)) => Err(format!(
                        "motion `{motion}` cannot run while tracking `{object}`: \
                         plans are baked in world coordinates, so release the track first"
                    )),
                    None => Ok(()),
                }
            }
            _ => Ok(()),
        }
    }

    /// A vehicle definition the scan engine can honour: a walkable path,
    /// resolvable stations and body obstacles, positive rates. Run once per
    /// simulate, so a broken definition fails with a message instead of a
    /// runtime surprise.
    fn validate_vehicle(&self, device: &Device) -> Result<(), String> {
        let DeviceKind::Vehicle {
            path,
            body,
            speed,
            turn_speed,
            start,
            tray,
            drive,
        } = &device.kind
        else {
            return Ok(());
        };
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
        held: &mut std::collections::HashMap<String, usize>,
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
            Action::StartRamp {
                robot,
                targets,
                duration,
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
                ..
            } => {
                let r = self.resolve_seq_robot(robot)?;
                if !self.obstacles.iter().any(|o| &o.name == object) {
                    return Err(format!("unknown obstacle `{object}`"));
                }
                if let Some(link) = link {
                    if self.robots()[r].model.link_index(link).is_none() {
                        return Err(format!("unknown link `{link}`"));
                    }
                }
                // One carrier at a time; a handover is written as
                // "detach → (place) → attach" (§design-multi-robot 6).
                if let Some(carrier) = held.get(object) {
                    return Err(format!(
                        "`{object}` is already attached to `{}`; detach it first \
                         (a handover is detach → attach)",
                        self.robots()[*carrier].name
                    ));
                }
                held.insert(object.clone(), r);
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
            } => {
                let r = self.resolve_seq_robot(robot)?;
                if !self.obstacles.iter().any(|o| &o.name == object) {
                    return Err(format!("unknown obstacle `{object}`"));
                }
                if held.contains_key(object) {
                    return Err(format!(
                        "`{object}` is already grasped; there is nothing to track"
                    ));
                }
                if let Some(link) = link {
                    if self.robots()[r].model.link_index(link).is_none() {
                        return Err(format!("unknown link `{link}`"));
                    }
                }
                Ok(())
            }
            Action::Untrack { robot } => self.resolve_seq_robot(robot).map(|_| ()),
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
                    (DeviceKind::LinearAxis { .. }, _) => {
                        Err(format!("axis `{device}` only takes move_to commands"))
                    }
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
                        "`{device}` is not a lift; move_to with a stop name drives a lift"
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
        }
    }
}
