//! PLC-style sequence authoring: a sequence is a list of *steps* (工程),
//! each firing entry actions and waiting on a transition condition — the
//! SFC / step-ladder mental model. The robot is one device among several:
//! motions are *started* by an action and *awaited* by a condition.
//!
//! Vocabulary mapping (see docs/design-sequence-control.md §3):
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
}

#[derive(Debug, Clone)]
pub enum DeviceCommand {
    Start,
    Stop,
    SetSpeed(f64),
    MoveTo(f64),
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
            | Action::Untrack { robot } => robot.as_mut(),
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
        Ok(())
    }

    pub fn set_sensors(&mut self, sensors: Vec<Sensor>) {
        self.sensors = sensors;
    }

    pub fn devices(&self) -> &[Device] {
        &self.devices
    }

    /// Adds or replaces an auxiliary device.
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
        Ok(())
    }

    pub fn set_devices(&mut self, devices: Vec<Device>) {
        self.devices = devices;
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
            Action::StartRamp { robot, .. } => self.resolve_seq_robot(robot).map(Some),
            _ => Ok(None),
        }
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
        // Tracking and grasping are modes the steps switch on and off, so
        // their rules are checked by walking the (linear) step list once —
        // per robot for tracking, per object for grasps.
        let mut tracked: Vec<Option<(String, Vec<String>)>> = vec![None; self.robots().len()];
        let mut held: std::collections::HashMap<String, usize> = self
            .attachments()
            .iter()
            .map(|a| (a.object.clone(), a.robot))
            .collect();
        for (i, step) in sequence.steps.iter().enumerate() {
            for action in &step.actions {
                self.validate_action(action, &mut held)
                    .map_err(|m| (Some(i), m))?;
                self.validate_tracking(action, &mut tracked)
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
                        return Err(format!("joint `{joint}` is not actuated"));
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
            Condition::Signal { name, .. } => {
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
                if !matches!(found.kind, DeviceKind::LinearAxis { .. }) {
                    return Err(format!(
                        "`device_done` waits for a linear axis; `{device}` is a conveyor"
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
