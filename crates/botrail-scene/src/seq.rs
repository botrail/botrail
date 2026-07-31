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
    /// Robot links trip it (light-curtain style).
    Robot,
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

#[derive(Debug, Clone)]
pub enum Action {
    /// Start a named motion (planned against the world state at this
    /// moment). Await it with [`Condition::Done`].
    StartMotion { motion: String },
    /// Linearly ramp a subset of joints (a gripper open/close) over
    /// `duration` seconds. Await it with [`Condition::Done`].
    StartRamp {
        targets: Vec<(String, f64)>,
        duration: f64,
    },
    /// Grasp: rigidly attach an obstacle at its current relative pose
    /// (defaults as in [`Scene::attach_obstacle`]). Instantaneous.
    Attach {
        object: String,
        link: Option<String>,
        touch_links: Option<Vec<String>>,
    },
    /// Release: the obstacle's pose freezes where it is. Instantaneous.
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
    /// The motion/ramp started by *this step* has finished.
    Done,
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

impl Condition {
    fn mentions_done(&self) -> bool {
        match self {
            Condition::Done => true,
            Condition::All(cs) | Condition::Any(cs) => cs.iter().any(|c| c.mentions_done()),
            _ => false,
        }
    }
}

impl Action {
    /// Does this action drive the robot's joint vector?
    pub(crate) fn drives_joints(&self) -> bool {
        matches!(self, Action::StartMotion { .. } | Action::StartRamp { .. })
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
        for (i, step) in sequence.steps.iter().enumerate() {
            let drivers = step.actions.iter().filter(|a| a.drives_joints()).count();
            if drivers > 1 {
                return Err((
                    Some(i),
                    "a step can start at most one motion or ramp (they both drive the same joints)"
                        .to_string(),
                ));
            }
            if step.transition.mentions_done() && drivers == 0 {
                return Err((
                    Some(i),
                    "`done` waits for a motion/ramp, but this step starts none".to_string(),
                ));
            }
            for action in &step.actions {
                self.validate_action(action).map_err(|m| (Some(i), m))?;
            }
            self.validate_condition(&step.transition)
                .map_err(|m| (Some(i), m))?;
        }
        Ok(())
    }

    fn validate_action(&self, action: &Action) -> Result<(), String> {
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
            Action::StartRamp { targets, duration } => {
                if targets.is_empty() {
                    return Err("ramp has no target joints".to_string());
                }
                if !(duration.is_finite() && *duration > 0.0) {
                    return Err(format!("ramp duration must be positive, got {duration}"));
                }
                for (joint, value) in targets {
                    let ji = self
                        .robot
                        .joint_index(joint)
                        .ok_or_else(|| format!("unknown joint `{joint}`"))?;
                    let j = &self.robot.joints[ji];
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
            Action::Attach { object, link, .. } => {
                if !self.obstacles.iter().any(|o| &o.name == object) {
                    return Err(format!("unknown obstacle `{object}`"));
                }
                if let Some(link) = link {
                    if self.robot.link_index(link).is_none() {
                        return Err(format!("unknown link `{link}`"));
                    }
                }
                Ok(())
            }
            Action::Detach { object } => {
                if !self.obstacles.iter().any(|o| &o.name == object) {
                    return Err(format!("unknown obstacle `{object}`"));
                }
                Ok(())
            }
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
