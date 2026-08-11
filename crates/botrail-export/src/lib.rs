//! Vendor robot-script export: a neutral program IR plus per-vendor script
//! backends (what OLP tools call "post-processors").
//!
//! A [`Program`] is a sparse sequence of move commands built from planned
//! path segments with [`build_program`]; a [`ScriptBackend`] renders it in
//! a vendor language (URScript today; TMScript / AUBO Lua / DENSO PAC are
//! planned). The crate is deliberately independent of botrail's scene and
//! motion types so it stays reusable as a generic post-processor library.
//!
//! Two principles, chosen to survive vendor differences:
//!
//! - **Targets stay joint-valued.** Every vendor language here can move
//!   linearly toward a joint-valued target; emitting Cartesian poses
//!   instead would drag in per-vendor orientation conventions and IK
//!   configuration flags, and could land on a different IK branch than the
//!   one that was planned.
//! - **Timing is the controller's job.** Commands carry physical bounds
//!   (rad, m, s) that backends convert; the robot re-parameterizes time
//!   itself, which keeps programs readable and compatible with vendor
//!   safety layers. Sample-faithful streaming export is a separate,
//!   per-vendor feature (out of scope here).

pub mod urscript;

use thiserror::Error;

#[derive(Debug, Error)]
pub enum ExportError {
    #[error("program has no move commands")]
    EmptyProgram,
    #[error("waypoint dof mismatch at index {index}: expected {expected}, got {got}")]
    WrongDof {
        index: usize,
        expected: usize,
        got: usize,
    },
    #[error("speed scale and joint velocity/acceleration limits must be positive")]
    NonPositiveSpeed,
    #[error("{dialect} supports {expected}-axis robots, got {got} joints")]
    UnsupportedDof {
        dialect: &'static str,
        expected: usize,
        got: usize,
    },
}

/// One program command. Blends are TCP-sphere radii in meters within which
/// the controller may round the corner into the next command; 0 stops
/// exactly at the target.
///
/// The I/O commands speak the least common denominator of industrial
/// controllers — numbered digital ports plus a level wait — which is what
/// a sequence's signals, device coils, and sensor contacts lower to. Port
/// numbering is the vendor's ("standard" digital I/O on UR).
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Joint-interpolated move. `velocity`/`acceleration` bound the
    /// leading axis (rad/s, rad/s²).
    MoveJoint {
        q: Vec<f64>,
        velocity: f64,
        acceleration: f64,
        blend: f64,
    },
    /// Straight-line TCP move toward a joint-valued target (the controller
    /// interpolates in Cartesian space itself). `velocity`/`acceleration`
    /// bound the TCP (m/s, m/s²).
    MoveLinear {
        q: Vec<f64>,
        velocity: f64,
        acceleration: f64,
        blend: f64,
    },
    /// Write a digital output (an output coil: gripper valve, conveyor
    /// run contact, weld fire).
    SetDigitalOut { port: u32, value: bool },
    /// Block until a digital input reads `value` (a level wait on an
    /// input contact: part-present beam, in-position feedback).
    WaitDigitalIn { port: u32, value: bool },
    /// Pause for a fixed time (an on-delay timer lowered to a wait).
    Sleep { seconds: f64 },
    /// Annotation carried into the script as a comment (step names,
    /// simulation-only actions). Never affects execution.
    Comment { text: String },
    /// Block until a compound digital test holds — a level wait over a
    /// boolean expression of inputs (`all_of`/`any_of` waits).
    WaitTest { test: DigitalTest },
    /// Selection divergence: block until any arm's test holds, then run
    /// exactly the first arm whose test does (authored order = priority)
    /// — an SFC branch lowered to `wait-any` + `if/elif`.
    Select { arms: Vec<SelectArm> },
}

/// One arm of a [`Command::Select`].
#[derive(Debug, Clone, PartialEq)]
pub struct SelectArm {
    pub test: DigitalTest,
    pub body: Vec<Command>,
}

/// A boolean expression over digital inputs — what branch guards lower
/// to. Kept minimal on purpose: whatever a controller cannot test as a
/// snapshot of its inputs (timers, edges) is refused upstream rather
/// than approximated here.
#[derive(Debug, Clone, PartialEq)]
pub enum DigitalTest {
    /// A digital input reads `value`.
    Input {
        port: u32,
        value: bool,
    },
    /// Always true — the guard was vacuous on a blocking controller
    /// (e.g. `done` when every move already blocks).
    Always,
    AllOf(Vec<DigitalTest>),
    AnyOf(Vec<DigitalTest>),
}

impl DigitalTest {
    /// Boolean identities, so `all_of(x, done)` renders as `x` rather
    /// than `(x and True)`: `Always` drops out of conjunctions, absorbs
    /// disjunctions, and one-element groups collapse.
    pub fn simplified(self) -> DigitalTest {
        match self {
            DigitalTest::AllOf(tests) => {
                let mut tests: Vec<DigitalTest> = tests
                    .into_iter()
                    .map(DigitalTest::simplified)
                    .filter(|t| !matches!(t, DigitalTest::Always))
                    .collect();
                match tests.len() {
                    0 => DigitalTest::Always,
                    1 => tests.remove(0),
                    _ => DigitalTest::AllOf(tests),
                }
            }
            DigitalTest::AnyOf(tests) => {
                let mut tests: Vec<DigitalTest> =
                    tests.into_iter().map(DigitalTest::simplified).collect();
                if tests.iter().any(|t| matches!(t, DigitalTest::Always)) {
                    return DigitalTest::Always;
                }
                match tests.len() {
                    0 => DigitalTest::Always,
                    1 => tests.remove(0),
                    _ => DigitalTest::AnyOf(tests),
                }
            }
            other => other,
        }
    }
}

/// A vendor-neutral robot program: named move sequence over a fixed joint
/// ordering. Backends map `joint_names` positionally onto the vendor's
/// axis order.
#[derive(Debug, Clone)]
pub struct Program {
    pub name: String,
    pub joint_names: Vec<String>,
    pub commands: Vec<Command>,
}

impl Program {
    pub fn dof(&self) -> usize {
        self.joint_names.len()
    }
}

/// How a [`PathSegment`] was planned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathKind {
    /// Free joint-space path: every waypoint matters.
    Joint,
    /// Straight TCP line: only the endpoints matter, the controller
    /// re-interpolates the line itself.
    Linear,
}

/// A planned path segment: sparse joint waypoints, both endpoints
/// included. Consecutive segments share their boundary waypoint.
#[derive(Debug, Clone)]
pub struct PathSegment {
    pub kind: PathKind,
    pub waypoints: Vec<Vec<f64>>,
}

#[derive(Debug, Clone)]
pub struct ProgramOptions {
    /// Scales every velocity and acceleration below (1 = the given limits).
    pub speed_scale: f64,
    /// Blend radius (m) for intermediate joint-path waypoints. Segment
    /// goals always stop exactly, matching the rest-to-rest model the
    /// trajectory was timed with. Keep 0 unless verified on the target
    /// controller: blends that overlap adjacent waypoints abort some
    /// controllers at runtime.
    pub blend_radius: f64,
    /// TCP velocity bound for linear moves (m/s).
    pub tcp_speed: f64,
    /// TCP acceleration bound for linear moves (m/s²).
    pub tcp_accel: f64,
    /// Emit an initial joint move to the first waypoint, making the
    /// program runnable from any (nearby, collision-free) configuration.
    pub move_to_start: bool,
}

impl Default for ProgramOptions {
    fn default() -> Self {
        ProgramOptions {
            speed_scale: 1.0,
            blend_radius: 0.0,
            tcp_speed: 0.25,
            tcp_accel: 1.2,
            move_to_start: true,
        }
    }
}

/// Builds a [`Program`] from planned path segments.
///
/// Joint moves are bounded by the *minimum* per-joint limit (scaled by
/// `speed_scale`): vendors interpret the value as the leading-axis bound,
/// so the minimum guarantees no joint exceeds its own limit. Linear
/// segments collapse to one `MoveLinear` per segment.
pub fn build_program(
    name: &str,
    joint_names: &[String],
    segments: &[PathSegment],
    joint_velocity_limits: &[f64],
    joint_acceleration_limits: &[f64],
    options: &ProgramOptions,
) -> Result<Program, ExportError> {
    let dof = joint_names.len();
    for (index, segment) in segments.iter().enumerate() {
        if let Some(q) = segment.waypoints.iter().find(|q| q.len() != dof) {
            return Err(ExportError::WrongDof {
                index,
                expected: dof,
                got: q.len(),
            });
        }
    }
    if options.speed_scale <= 0.0
        || options.tcp_speed <= 0.0
        || options.tcp_accel <= 0.0
        || joint_velocity_limits.len() != dof
        || joint_acceleration_limits.len() != dof
        || joint_velocity_limits.iter().any(|v| *v <= 0.0)
        || joint_acceleration_limits.iter().any(|a| *a <= 0.0)
    {
        return Err(ExportError::NonPositiveSpeed);
    }

    let fold_min = |xs: &[f64]| xs.iter().copied().fold(f64::INFINITY, f64::min);
    let joint_velocity = fold_min(joint_velocity_limits) * options.speed_scale;
    let joint_acceleration = fold_min(joint_acceleration_limits) * options.speed_scale;
    let tcp_velocity = options.tcp_speed * options.speed_scale;
    let tcp_acceleration = options.tcp_accel * options.speed_scale;

    let mut commands = Vec::new();
    if options.move_to_start {
        if let Some(first) = segments.iter().find_map(|s| s.waypoints.first()) {
            commands.push(Command::MoveJoint {
                q: first.clone(),
                velocity: joint_velocity,
                acceleration: joint_acceleration,
                blend: 0.0,
            });
        }
    }
    for segment in segments {
        match segment.kind {
            PathKind::Joint => {
                let last = segment.waypoints.len().saturating_sub(1);
                for (i, q) in segment.waypoints.iter().enumerate().skip(1) {
                    commands.push(Command::MoveJoint {
                        q: q.clone(),
                        velocity: joint_velocity,
                        acceleration: joint_acceleration,
                        blend: if i < last { options.blend_radius } else { 0.0 },
                    });
                }
            }
            PathKind::Linear => {
                if segment.waypoints.len() > 1 {
                    commands.push(Command::MoveLinear {
                        q: segment.waypoints.last().expect("len > 1").clone(),
                        velocity: tcp_velocity,
                        acceleration: tcp_acceleration,
                        blend: 0.0,
                    });
                }
            }
        }
    }
    if commands.is_empty() {
        return Err(ExportError::EmptyProgram);
    }
    Ok(Program {
        name: name.to_string(),
        joint_names: joint_names.to_vec(),
        commands,
    })
}

/// Renders a [`Program`] in one vendor script language.
pub trait ScriptBackend {
    /// Stable identifier, e.g. `"urscript"`.
    fn dialect(&self) -> &'static str;
    /// Conventional file extension without the dot, e.g. `"script"`.
    fn file_extension(&self) -> &'static str;
    fn emit(&self, program: &Program) -> Result<String, ExportError>;
}

/// Dialect identifiers [`backend`] resolves.
pub const DIALECTS: &[&str] = &["urscript"];

/// Looks up a backend by dialect identifier (see [`DIALECTS`]).
pub fn backend(dialect: &str) -> Option<Box<dyn ScriptBackend>> {
    match dialect {
        "urscript" => Some(Box::new(urscript::UrScript)),
        _ => None,
    }
}

/// Formats with at most 6 decimals, trimming trailing zeros
/// ("1.500000" → "1.5", "0.000000" → "0").
pub(crate) fn fmt_num(x: f64) -> String {
    let mut s = format!("{x:.6}");
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    if s == "-0" {
        "0".to_string()
    } else {
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names(dof: usize) -> Vec<String> {
        (0..dof).map(|i| format!("j{i}")).collect()
    }

    fn limits(dof: usize) -> (Vec<f64>, Vec<f64>) {
        (vec![2.0; dof], vec![4.0; dof])
    }

    #[test]
    fn boundary_waypoints_are_not_duplicated() {
        let a = vec![0.0, 0.0];
        let b = vec![0.5, -0.2];
        let c = vec![1.0, 0.3];
        let segments = vec![
            PathSegment {
                kind: PathKind::Joint,
                waypoints: vec![a.clone(), b.clone()],
            },
            PathSegment {
                kind: PathKind::Joint,
                waypoints: vec![b.clone(), c.clone()],
            },
        ];
        let (vel, acc) = limits(2);
        let program = build_program(
            "p",
            &names(2),
            &segments,
            &vel,
            &acc,
            &ProgramOptions::default(),
        )
        .unwrap();
        // Move-to-start(a), then b, then c: the shared boundary b appears once.
        let targets: Vec<&Vec<f64>> = program
            .commands
            .iter()
            .map(|c| match c {
                Command::MoveJoint { q, .. } | Command::MoveLinear { q, .. } => q,
                other => panic!("build_program emits only moves, got {other:?}"),
            })
            .collect();
        assert_eq!(targets, vec![&a, &b, &c]);
    }

    #[test]
    fn blend_only_on_intermediate_waypoints() {
        let segments = vec![PathSegment {
            kind: PathKind::Joint,
            waypoints: vec![vec![0.0], vec![0.3], vec![0.6], vec![1.0]],
        }];
        let options = ProgramOptions {
            blend_radius: 0.05,
            move_to_start: false,
            ..ProgramOptions::default()
        };
        let program = build_program("p", &names(1), &segments, &[2.0], &[4.0], &options).unwrap();
        let blends: Vec<f64> = program
            .commands
            .iter()
            .map(|c| match c {
                Command::MoveJoint { blend, .. } | Command::MoveLinear { blend, .. } => *blend,
                other => panic!("build_program emits only moves, got {other:?}"),
            })
            .collect();
        // Intermediates blend, the segment goal stops exactly.
        assert_eq!(blends, vec![0.05, 0.05, 0.0]);
    }

    #[test]
    fn linear_segments_collapse_to_one_move() {
        let follow: Vec<Vec<f64>> = (0..=10).map(|k| vec![k as f64 * 0.1, 0.0]).collect();
        let goal = follow.last().unwrap().clone();
        let segments = vec![PathSegment {
            kind: PathKind::Linear,
            waypoints: follow,
        }];
        let (vel, acc) = limits(2);
        let options = ProgramOptions {
            speed_scale: 0.5,
            ..ProgramOptions::default()
        };
        let program = build_program("p", &names(2), &segments, &vel, &acc, &options).unwrap();
        assert_eq!(program.commands.len(), 2); // move-to-start + one movel
        match &program.commands[1] {
            Command::MoveLinear {
                q,
                velocity,
                acceleration,
                blend,
            } => {
                assert_eq!(q, &goal);
                assert!((velocity - 0.125).abs() < 1e-12); // 0.25 * 0.5
                assert!((acceleration - 0.6).abs() < 1e-12); // 1.2 * 0.5
                assert_eq!(*blend, 0.0);
            }
            other => panic!("expected MoveLinear, got {other:?}"),
        }
    }

    #[test]
    fn joint_speed_is_min_limit_scaled() {
        let segments = vec![PathSegment {
            kind: PathKind::Joint,
            waypoints: vec![vec![0.0, 0.0], vec![1.0, 1.0]],
        }];
        let options = ProgramOptions {
            speed_scale: 0.5,
            move_to_start: false,
            ..ProgramOptions::default()
        };
        let program = build_program(
            "p",
            &names(2),
            &segments,
            &[2.0, 3.0],
            &[4.0, 6.0],
            &options,
        )
        .unwrap();
        match &program.commands[0] {
            Command::MoveJoint {
                velocity,
                acceleration,
                ..
            } => {
                assert!((velocity - 1.0).abs() < 1e-12); // min(2,3) * 0.5
                assert!((acceleration - 2.0).abs() < 1e-12); // min(4,6) * 0.5
            }
            other => panic!("expected MoveJoint, got {other:?}"),
        }
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        let (vel, acc) = limits(2);
        let joint = |wps: Vec<Vec<f64>>| PathSegment {
            kind: PathKind::Joint,
            waypoints: wps,
        };
        assert!(matches!(
            build_program("p", &names(2), &[], &vel, &acc, &ProgramOptions::default()),
            Err(ExportError::EmptyProgram)
        ));
        assert!(matches!(
            build_program(
                "p",
                &names(2),
                &[joint(vec![vec![0.0]])],
                &vel,
                &acc,
                &ProgramOptions::default()
            ),
            Err(ExportError::WrongDof { .. })
        ));
        assert!(matches!(
            build_program(
                "p",
                &names(2),
                &[joint(vec![vec![0.0, 0.0], vec![1.0, 1.0]])],
                &vel,
                &acc,
                &ProgramOptions {
                    speed_scale: 0.0,
                    ..ProgramOptions::default()
                }
            ),
            Err(ExportError::NonPositiveSpeed)
        ));
        // A single-waypoint segment produces no motion at all.
        assert!(matches!(
            build_program(
                "p",
                &names(2),
                &[joint(vec![vec![0.0, 0.0]])],
                &vel,
                &acc,
                &ProgramOptions {
                    move_to_start: false,
                    ..ProgramOptions::default()
                }
            ),
            Err(ExportError::EmptyProgram)
        ));
    }

    #[test]
    fn fmt_num_trims_trailing_zeros() {
        assert_eq!(fmt_num(0.0), "0");
        assert_eq!(fmt_num(-0.0000001), "0");
        assert_eq!(fmt_num(1.5), "1.5");
        assert_eq!(fmt_num(-0.123456), "-0.123456");
        assert_eq!(fmt_num(2.0), "2");
    }
}
