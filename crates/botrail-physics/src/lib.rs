//! Engine-independent physics vocabulary for botrail.
//!
//! This crate is deliberately small: the authored per-obstacle properties
//! ([`BodyProps`]), a lowered world description ([`WorldDesc`]) built by the
//! rollout, and the [`PhysicsBackend`] trait an engine adapter implements
//! (`botrail-physics-rapier` is the first). The trait speaks *what the
//! rollout needs*, not any engine's full surface — that narrowness is what
//! keeps a second backend (MuJoCo, GPU) possible later.
//!
//! Math boundary: everything here speaks botrail's nalgebra
//! (`Isometry3<f64>`) plus parry shapes (`SharedShape`, shared with
//! `botrail-collide` — the same crate version, so collision assets convert
//! for free). An adapter owns whatever conversion its engine needs.

use nalgebra::{Isometry3, Vector3};
use parry3d_f64::math::Pose as PartPose;
use parry3d_f64::shape::SharedShape;

#[derive(Debug, thiserror::Error)]
pub enum PhysicsError {
    #[error("physics backend error: {0}")]
    Backend(String),
}

/// Who owns a body's pose during a physics rollout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BodyKind {
    /// Fixed world geometry (the default — exactly today's obstacle).
    #[default]
    Static,
    /// Pose decided by the existing rollout logic (robot links, device-
    /// driven obstacles) and *supplied* to the physics world each tick.
    Kinematic,
    /// Pose decided by the physics engine and read back by the rollout.
    Dynamic,
}

/// Contact response of a body's surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhysicsMaterial {
    pub friction: f64,
    pub restitution: f64,
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        PhysicsMaterial {
            friction: 0.5,
            restitution: 0.0,
        }
    }
}

/// Density used when a dynamic body names no mass: water/plastic-ish, so an
/// unannotated workpiece neither floats away nor anchors the cell.
pub const DEFAULT_DENSITY: f64 = 1000.0;

/// Authored physics properties of one obstacle. Absent means "exactly
/// today's obstacle" — and even when present, the properties are inert
/// unless the bake runs with a physics backend.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyProps {
    pub kind: BodyKind,
    /// Total mass in kg; `None` derives it from the collision shape's
    /// volume at [`DEFAULT_DENSITY`]. Center of mass and inertia always
    /// come from the collision shape (explicit tensors are a later,
    /// dynamic-robot-era vocabulary).
    pub mass: Option<f64>,
    pub material: PhysicsMaterial,
    pub linear_damping: f64,
    /// Small default so settled parts stop ringing instead of jittering
    /// on their corners.
    pub angular_damping: f64,
    /// Continuous collision detection, for small fast parts.
    pub ccd: bool,
    /// Explicit mass properties (a robot link's URDF `<inertial>`); when
    /// present they replace the shape-derived mass, center and inertia
    /// entirely (`mass` is ignored). Design-rl-dynamics.md RD0.
    pub mass_properties: Option<MassProperties>,
}

/// A body's mass properties stated outright: the mass, the center of
/// mass in the body frame, and the inertia tensor about that center in
/// the body frame's axes.
#[derive(Debug, Clone, PartialEq)]
pub struct MassProperties {
    pub mass: f64,
    pub com: Vector3<f64>,
    pub inertia: nalgebra::Matrix3<f64>,
}

impl Default for BodyProps {
    fn default() -> Self {
        BodyProps {
            kind: BodyKind::Static,
            mass: None,
            material: PhysicsMaterial::default(),
            linear_damping: 0.0,
            angular_damping: 0.05,
            ccd: false,
            mass_properties: None,
        }
    }
}

impl BodyProps {
    /// The usual authoring entry: a dynamic body with defaults.
    pub fn dynamic() -> Self {
        BodyProps {
            kind: BodyKind::Dynamic,
            ..Default::default()
        }
    }
}

/// Index of a body within the [`WorldDesc`] it was created from:
/// `BodyId(i)` is `world.bodies[i]`. The rollout keeps its own mapping
/// from ids to scene entities.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyId(pub u32);

/// One rigid body of the lowered world: a pose, the collision parts
/// (exactly the shapes `botrail-collide` built — primitives and VHACD
/// compounds), and the authored properties.
#[derive(Clone)]
pub struct BodyDesc {
    pub kind: BodyKind,
    pub pose: Isometry3<f64>,
    /// Local shape parts `(part pose, shape)` in parry's own math, shared
    /// with the collision checker.
    pub parts: Vec<(PartPose, SharedShape)>,
    pub props: BodyProps,
    /// Self-collision group: bodies sharing a nonzero group never collide
    /// with each other. The lowering gives every link of one robot the
    /// same group, so a dynamic finger doesn't fight its own palm mirror —
    /// for all-kinematic robots this changes nothing (kinematic pairs are
    /// never solved). `0` (the default) collides with everything.
    pub group: u32,
}

/// The kinematic kind of a driven joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
    /// A weld: all six axes locked, no motor. What ties a fixed-jointed
    /// link inside a finger subtree (an outer finger bar, a rubber pad)
    /// to its moving carrier once both are dynamic bodies.
    Fixed,
}

/// A force-capped position motor: the drive of a gripper finger. The
/// realized clamp at rest is what the contact develops at the standing
/// penetration the commanded overtravel buys; `max_force` is the ceiling
/// (N for prismatic, N·m for revolute), which is also what a too-heavy
/// load defeats.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointMotor {
    pub stiffness: f64,
    pub damping: f64,
    pub max_force: f64,
}

/// One driven joint of the lowered world, connecting two of its bodies.
/// Anchors give the joint frame on each body (`local1` on `parent`,
/// `local2` on `child`); `axis` is the free axis in the joint frame.
/// [`JointKind::Fixed`] welds ignore `axis`, `limits` and `motor`, and
/// must come AFTER every motored joint in [`WorldDesc::joints`] — the
/// rollout addresses motored joints by index.
#[derive(Clone)]
pub struct JointDesc {
    pub parent: BodyId,
    pub child: BodyId,
    pub kind: JointKind,
    pub local1: Isometry3<f64>,
    pub local2: Isometry3<f64>,
    pub axis: Vector3<f64>,
    /// Position limits `(lower, upper)`; `None` leaves the axis unbounded.
    pub limits: Option<(f64, f64)>,
    pub motor: JointMotor,
}

/// A conveyor's contract with the physics world: contact points inside
/// this box, between a dynamic body and static/kinematic scenery, are
/// driven at `velocity` as if the surface under them were a moving belt
/// (a solver-contact tangent velocity — rapier's own conveyor mechanism).
/// The box is the *authored conveyor zone*, unchanged; dynamic-dynamic
/// contacts are left alone, so a part stacked on a carried part rides
/// friction, not the belt.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceVelocityZone {
    pub pose: Isometry3<f64>,
    pub half_extents: Vector3<f64>,
    /// Belt surface velocity, world frame (m/s).
    pub velocity: Vector3<f64>,
    /// A stopped belt leaves its contacts to ordinary friction.
    pub active: bool,
}

/// The lowered world a backend resets to. `bodies` is in a fixed,
/// scene-derived order — creation order is part of the determinism
/// contract, so the lowering never reorders it. `zones` is likewise in
/// authoring order; [`PhysicsBackend::set_zone`] addresses them by index.
#[derive(Clone, Default)]
pub struct WorldDesc {
    /// Gravity in m/s²; botrail is z-up, so the default points down z.
    pub gravity: Vector3<f64>,
    pub bodies: Vec<BodyDesc>,
    pub zones: Vec<SurfaceVelocityZone>,
    /// Driven joints (a friction-grasp gripper's fingers), in authoring
    /// order; [`PhysicsBackend::set_joint_target`] addresses them by index.
    pub joints: Vec<JointDesc>,
}

impl WorldDesc {
    pub fn new() -> Self {
        WorldDesc {
            gravity: Vector3::new(0.0, 0.0, -9.81),
            bodies: Vec::new(),
            zones: Vec::new(),
            joints: Vec::new(),
        }
    }
}

/// One scan tick's raw contact happenings, drained after stepping. Only
/// pairs involving a dynamic body report (the event flags ride on dynamic
/// colliders); the rollout assembles these into touch episodes
/// (`ContactSpan`s) with wall-clock times and names.
#[derive(Debug, Clone, Default)]
pub struct TickContacts {
    /// Pairs that began touching, with a representative world-space
    /// contact point (the flash position).
    pub started: Vec<(BodyId, BodyId, Vector3<f64>)>,
    /// Pairs that stopped touching.
    pub stopped: Vec<(BodyId, BodyId)>,
    /// Total contact-force magnitudes (N) observed this tick for touching
    /// pairs, one entry per substep report — the rollout keeps the peak.
    pub forces: Vec<(BodyId, BodyId, f64)>,
}

/// A rigid-body velocity, for handing a released object its carrier's
/// motion (detach) — linear and angular parts in world coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Velocity {
    pub linear: Vector3<f64>,
    pub angular: Vector3<f64>,
}

/// What the rollout needs from a physics engine — nothing more. One
/// backend instance serves one rollout: `reset` builds the world, then the
/// scan loop alternates kinematic supply, `step`, and pose read-back.
pub trait PhysicsBackend: Send + Sync {
    /// Engine name for timeline self-description (e.g. `"rapier"`).
    fn name(&self) -> &'static str;

    /// Rebuilds the physics world from a lowered description, dropping
    /// whatever was there. Bodies are created in `world.bodies` order.
    fn reset(&mut self, world: &WorldDesc) -> Result<(), PhysicsError>;

    /// Supplies a kinematic body's target pose for the next step(s); the
    /// engine derives its velocity for contact resolution.
    fn set_kinematic_pose(&mut self, body: BodyId, pose: Isometry3<f64>);

    /// Switches a body between kinematic and dynamic (attach / detach).
    /// `velocity` seeds the dynamic body's motion on release; `None`
    /// releases it at rest.
    fn set_body_kind(&mut self, body: BodyId, kind: BodyKind, velocity: Option<Velocity>);

    /// Moves a body into self-collision group `group` (`0`: collides with
    /// everything) — a part taken in hand joins its carrier's group so it
    /// rides between the fingers without fighting them, and leaves it
    /// when let go.
    fn set_body_group(&mut self, body: BodyId, group: u32);

    /// Updates one surface-velocity zone (a conveyor's per-tick state:
    /// current belt velocity, running or stopped). `zone` indexes
    /// [`WorldDesc::zones`]; geometry never changes, only the drive.
    fn set_zone(&mut self, zone: usize, velocity: Vector3<f64>, active: bool);

    /// Advances the world by one substep of `dt` seconds.
    fn step(&mut self, dt: f64);

    /// Current world pose of a body.
    fn body_pose(&self, body: BodyId) -> Isometry3<f64>;

    /// Whether the engine has put this body to sleep (at rest). The
    /// rollout folds sleeping stretches into `Hold` spans.
    fn is_sleeping(&self, body: BodyId) -> bool;

    /// Current world-frame velocity of a body: the engine's for a
    /// dynamic body, the supplied pose difference for a kinematic one.
    /// An observation channel (design-rl.md §3.3), never a bake input.
    fn body_velocity(&self, body: BodyId) -> Velocity;

    /// Whether the engine simulates `body` as a free (dynamic) body right
    /// now — what a hand can move. A mirror, a part taken in hand, a
    /// static body: `false`.
    fn is_dynamic_body(&self, body: BodyId) -> bool;

    /// The mass the engine simulates `body` with (kg).
    fn body_mass(&self, body: BodyId) -> f64;

    /// Sets the external force (N) applied at world `point` on `body`
    /// every step until [`clear_force`](Self::clear_force) — a hand on
    /// the body: a mouse pick, a push (design-physics-pick.md). Replaces
    /// the force set before on that body; a body that is not dynamic
    /// takes none.
    fn set_force_at(&mut self, body: BodyId, force: Vector3<f64>, point: Vector3<f64>);

    /// Lifts the external force from `body`.
    fn clear_force(&mut self, body: BodyId);

    /// Takes the contact events accumulated since the last drain (the
    /// substeps of one scan tick, in practice).
    fn drain_contacts(&mut self) -> TickContacts;

    /// Supplies a driven joint's position target for the next step(s).
    /// `joint` indexes [`WorldDesc::joints`].
    fn set_joint_target(&mut self, joint: usize, position: f64);

    /// Current position of a driven joint (its coordinate along/about the
    /// joint axis) — what the rollout writes back into the baked track so
    /// a stalled finger plays back where it really stopped.
    fn joint_position(&self, joint: usize) -> f64;

    /// Commands a velocity through driven joint `joint`'s motor instead
    /// of a position: the motor's damping is the velocity loop's gain,
    /// its cap the ceiling, and the position target is dropped. A dynamic
    /// robot's servo (design-rl-dynamics.md) is a position loop the
    /// rollout closes over this velocity loop.
    fn set_joint_velocity(&mut self, joint: usize, velocity: f64);

    /// Puts driven joint `joint` under a raw torque (N·m; N for a
    /// prismatic joint): its position motor is switched off and `torque`
    /// acts on the joint every step until changed. `None` hands the joint
    /// back to its motor at whatever target it holds. Design-rl-dynamics.md
    /// RD0 — the `Torque` action of a dynamic robot.
    fn set_joint_torque(&mut self, joint: usize, torque: Option<f64>);
}
