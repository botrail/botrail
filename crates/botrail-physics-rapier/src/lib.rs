//! Rapier adapter for `botrail-physics`.
//!
//! rapier3d-f64 0.34 shares parry3d-f64 0.29 with the workspace, so the
//! collision parts a [`WorldDesc`] carries (`SharedShape` — primitives and
//! VHACD compounds from `botrail-collide`) become rapier colliders without
//! any shape conversion. Poses cross the boundary by components, the same
//! way `botrail-collide` talks to parry (its `convert.rs`); rapier's own
//! math *is* parry's (glamx), so [`to_pose`]/[`from_pose`] are the whole
//! boundary.
//!
//! Determinism: bodies are created in `WorldDesc` order, stepping is
//! single-threaded (no `parallel` feature), and the timestep is whatever
//! the rollout hands `step` — under those conditions rapier is
//! deterministic on a given machine and build, which is the guarantee
//! botrail documents for physics bakes (design-physics.md 判断 D10).

use std::collections::HashMap;
use std::sync::Mutex;

use botrail_physics::{
    BodyId, BodyKind, PhysicsBackend, PhysicsError, TickContacts, Velocity, WorldDesc,
    DEFAULT_DENSITY,
};
use nalgebra::Isometry3;
use rapier3d_f64::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodyBuilder, RigidBodyHandle, RigidBodySet, RigidBodyType,
};
use rapier3d_f64::geometry::{
    BroadPhaseBvh, ColliderBuilder, ColliderSet, CollisionEvent, ContactPair, NarrowPhase,
};
use rapier3d_f64::math::{Pose, Real, Rotation, Vector};
use rapier3d_f64::pipeline::{
    ActiveEvents, ActiveHooks, ContactModificationContext, EventHandler, PhysicsHooks,
    PhysicsPipeline,
};

/// nalgebra (workspace 0.33) → rapier/parry math, by components.
fn to_pose(iso: &Isometry3<f64>) -> Pose {
    let t = iso.translation;
    let q = iso.rotation.coords;
    Pose {
        rotation: Rotation::from_xyzw(q.x, q.y, q.z, q.w),
        translation: Vector::new(t.x, t.y, t.z),
    }
}

fn from_pose(pose: &Pose) -> Isometry3<f64> {
    let q = pose.rotation;
    let t = pose.translation;
    Isometry3::from_parts(
        nalgebra::Translation3::new(t.x, t.y, t.z),
        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q.w, q.x, q.y, q.z)),
    )
}

/// Contact points may sit a hair below the surface they rest on
/// (solver slop), so a zone authored with its lower face exactly on the
/// belt surface — the natural authoring — must still catch them. The
/// containment test inflates the box by this much on every side.
const ZONE_MARGIN: f64 = 0.005;

/// One belt zone, in rapier math, ready for the per-contact test.
struct ZoneState {
    inv_pose: Pose,
    /// Authored half extents plus [`ZONE_MARGIN`].
    half: Vector,
    velocity: Vector,
    active: bool,
}

/// The conveyor mechanism (design-physics.md 判断 D7): for a contact
/// between a dynamic body and static/kinematic scenery whose point lies
/// in an active zone, the solver is asked to drive the tangential
/// relative velocity to the belt velocity — rapier's own conveyor idiom.
/// `tangent_velocity` is the desired velocity of *body 2 relative to
/// body 1* (pinned empirically by `a_belt_conveys_at_its_surface_velocity`,
/// which drives the box both ways round the pair), so the sign flips with
/// which side of the pair the dynamic body landed on.
#[derive(Default)]
struct ConveyorHooks {
    zones: Vec<ZoneState>,
}

impl PhysicsHooks for ConveyorHooks {
    fn modify_solver_contacts(&self, ctx: &mut ContactModificationContext) {
        let dynamic1 = ctx
            .rigid_body1
            .is_some_and(|h| ctx.bodies[h].is_dynamic());
        let dynamic2 = ctx
            .rigid_body2
            .is_some_and(|h| ctx.bodies[h].is_dynamic());
        // Dynamic-dynamic rides friction (a part on a carried part), and
        // scenery-scenery never solves contacts anyway.
        if dynamic1 == dynamic2 {
            return;
        }
        let sign = if dynamic1 { -1.0 } else { 1.0 };
        for contact in ctx.solver_contacts.iter_mut() {
            for zone in &self.zones {
                if !zone.active {
                    continue;
                }
                let local = zone.inv_pose.transform_point(contact.point);
                if local.x.abs() <= zone.half.x
                    && local.y.abs() <= zone.half.y
                    && local.z.abs() <= zone.half.z
                {
                    contact.tangent_velocity = zone.velocity * sign;
                    break;
                }
            }
        }
    }
}

/// A drained-later event sink (rapier calls the handler with `&self`, so
/// the buffer lives behind a mutex — stepping is single-threaded here,
/// the lock is uncontended).
#[derive(Default)]
struct ContactCollector {
    events: Mutex<Vec<RawContact>>,
}

enum RawContact {
    Started(RigidBodyHandle, RigidBodyHandle, Vector),
    Stopped(RigidBodyHandle, RigidBodyHandle),
    Force(RigidBodyHandle, RigidBodyHandle, f64),
}

impl ContactCollector {
    /// Body handles of a collider pair; `None` drops the event (a
    /// collider without a body never occurs here — every collider is
    /// inserted with a parent).
    fn bodies(
        colliders: &ColliderSet,
        c1: rapier3d_f64::geometry::ColliderHandle,
        c2: rapier3d_f64::geometry::ColliderHandle,
    ) -> Option<(RigidBodyHandle, RigidBodyHandle)> {
        Some((
            colliders.get(c1)?.parent()?,
            colliders.get(c2)?.parent()?,
        ))
    }
}

impl EventHandler for ContactCollector {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        event: CollisionEvent,
        contact_pair: Option<&ContactPair>,
    ) {
        let Some((b1, b2)) = Self::bodies(colliders, event.collider1(), event.collider2())
        else {
            return;
        };
        let raw = match event {
            CollisionEvent::Started(c1, _, _) => {
                // A representative world contact point for the flash: the
                // first manifold point, or the collider's own position
                // when the pair reports none.
                let point = contact_pair
                    .and_then(|pair| {
                        let manifold = pair.manifolds.first()?;
                        let local = manifold.points.first()?.local_p1;
                        let collider = colliders.get(c1)?;
                        Some(collider.position().transform_point(local))
                    })
                    .or_else(|| colliders.get(c1).map(|c| c.position().translation));
                RawContact::Started(b1, b2, point.unwrap_or(Vector::ZERO))
            }
            CollisionEvent::Stopped(..) => RawContact::Stopped(b1, b2),
        };
        self.events.lock().expect("collector poisoned").push(raw);
    }

    fn handle_contact_force_event(
        &self,
        _dt: Real,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let Some((b1, b2)) =
            Self::bodies(colliders, contact_pair.collider1, contact_pair.collider2)
        else {
            return;
        };
        self.events
            .lock()
            .expect("collector poisoned")
            .push(RawContact::Force(b1, b2, total_force_magnitude));
    }
}

/// One rapier world serving one rollout.
pub struct RapierBackend {
    gravity: Vector,
    params: IntegrationParameters,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd: CCDSolver,
    /// `BodyId(i)` → handle, in `WorldDesc` order.
    handles: Vec<RigidBodyHandle>,
    /// The reverse map, for naming contact events.
    body_ids: HashMap<RigidBodyHandle, BodyId>,
    hooks: ConveyorHooks,
    collector: ContactCollector,
}

impl Default for RapierBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl RapierBackend {
    pub fn new() -> Self {
        RapierBackend {
            gravity: Vector::new(0.0, 0.0, -9.81),
            params: IntegrationParameters::default(),
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
            handles: Vec::new(),
            body_ids: HashMap::new(),
            hooks: ConveyorHooks::default(),
            collector: ContactCollector::default(),
        }
    }

    fn body(&self, id: BodyId) -> &rapier3d_f64::dynamics::RigidBody {
        &self.bodies[self.handles[id.0 as usize]]
    }

    fn body_mut(&mut self, id: BodyId) -> &mut rapier3d_f64::dynamics::RigidBody {
        &mut self.bodies[self.handles[id.0 as usize]]
    }
}

impl PhysicsBackend for RapierBackend {
    fn name(&self) -> &'static str {
        "rapier"
    }

    fn reset(&mut self, world: &WorldDesc) -> Result<(), PhysicsError> {
        *self = RapierBackend::new();
        self.gravity = Vector::new(world.gravity.x, world.gravity.y, world.gravity.z);
        self.hooks.zones = world
            .zones
            .iter()
            .map(|z| ZoneState {
                inv_pose: to_pose(&z.pose).inverse(),
                half: Vector::new(
                    z.half_extents.x + ZONE_MARGIN,
                    z.half_extents.y + ZONE_MARGIN,
                    z.half_extents.z + ZONE_MARGIN,
                ),
                velocity: Vector::new(z.velocity.x, z.velocity.y, z.velocity.z),
                active: z.active,
            })
            .collect();
        for desc in &world.bodies {
            let builder = match desc.kind {
                BodyKind::Static => RigidBodyBuilder::fixed(),
                BodyKind::Kinematic => RigidBodyBuilder::kinematic_position_based(),
                BodyKind::Dynamic => RigidBodyBuilder::dynamic(),
            };
            let body = builder
                .pose(to_pose(&desc.pose))
                .linear_damping(desc.props.linear_damping)
                .angular_damping(desc.props.angular_damping)
                .ccd_enabled(desc.props.ccd)
                .build();
            let handle = self.bodies.insert(body);
            self.body_ids.insert(handle, BodyId(self.handles.len() as u32));
            self.handles.push(handle);
            // An authored mass spreads over the parts as a uniform density,
            // so the center of mass and inertia still come from the shape.
            let density = match desc.props.mass {
                Some(mass) => {
                    let volume: f64 = desc
                        .parts
                        .iter()
                        .map(|(_, shape)| shape.mass_properties(1.0).mass())
                        .sum();
                    if volume > 1e-12 {
                        mass / volume
                    } else {
                        DEFAULT_DENSITY
                    }
                }
                None => DEFAULT_DENSITY,
            };
            for (part_pose, shape) in &desc.parts {
                let mut collider = ColliderBuilder::new(shape.clone())
                    .position(*part_pose)
                    .friction(desc.props.material.friction)
                    .restitution(desc.props.material.restitution)
                    .density(density);
                // The belt hook runs for pairs with a flagged collider;
                // flagging the dynamic side covers every contact a belt
                // could drive, and nothing else pays for the callback.
                if desc.kind == BodyKind::Dynamic && !self.hooks.zones.is_empty() {
                    collider = collider.active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS);
                }
                // Contact recording rides the dynamic side the same way:
                // begin/end events plus per-step force reports (threshold
                // zero — the rollout keeps only each episode's peak).
                if desc.kind == BodyKind::Dynamic {
                    collider = collider
                        .active_events(
                            ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS,
                        )
                        .contact_force_event_threshold(0.0);
                }
                self.colliders
                    .insert_with_parent(collider.build(), handle, &mut self.bodies);
            }
        }
        Ok(())
    }

    fn set_kinematic_pose(&mut self, body: BodyId, pose: Isometry3<f64>) {
        self.body_mut(body).set_next_kinematic_position(to_pose(&pose));
    }

    fn set_body_kind(&mut self, body: BodyId, kind: BodyKind, velocity: Option<Velocity>) {
        let rb = self.body_mut(body);
        let body_type = match kind {
            BodyKind::Static => RigidBodyType::Fixed,
            BodyKind::Kinematic => RigidBodyType::KinematicPositionBased,
            BodyKind::Dynamic => RigidBodyType::Dynamic,
        };
        rb.set_body_type(body_type, true);
        if let Some(v) = velocity {
            rb.set_linvel(Vector::new(v.linear.x, v.linear.y, v.linear.z), true);
            rb.set_angvel(Vector::new(v.angular.x, v.angular.y, v.angular.z), true);
        }
    }

    fn set_zone(&mut self, zone: usize, velocity: nalgebra::Vector3<f64>, active: bool) {
        let z = &mut self.hooks.zones[zone];
        z.velocity = Vector::new(velocity.x, velocity.y, velocity.z);
        z.active = active;
    }

    fn step(&mut self, dt: f64) {
        // A belt at typical conveyor speed sits *below* rapier's default
        // sleep threshold (0.4 length-units/s), so a smoothly carried part
        // would doze off mid-belt and freeze (observed: constant 0.3 m/s
        // cruise slept after ~2 s). Keep dynamic bodies whose center is
        // inside an active zone awake — the same center-in-box rule the
        // kinematic advection captures by — and let ordinary sleeping
        // resume the moment the belt stops.
        if self.hooks.zones.iter().any(|z| z.active) {
            for &handle in &self.handles {
                let body = &mut self.bodies[handle];
                if !body.is_dynamic() {
                    continue;
                }
                let p = body.position().translation;
                let carried = self.hooks.zones.iter().any(|z| {
                    if !z.active {
                        return false;
                    }
                    let local = z.inv_pose.transform_point(p);
                    local.x.abs() <= z.half.x
                        && local.y.abs() <= z.half.y
                        && local.z.abs() <= z.half.z
                });
                if carried {
                    body.wake_up(true);
                }
            }
        }
        self.params.dt = dt;
        self.pipeline.step(
            self.gravity,
            &self.params,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            &self.hooks,
            &self.collector,
        );
    }

    fn body_pose(&self, body: BodyId) -> Isometry3<f64> {
        from_pose(self.body(body).position())
    }

    fn is_sleeping(&self, body: BodyId) -> bool {
        self.body(body).is_sleeping()
    }

    fn drain_contacts(&mut self) -> TickContacts {
        let raw = std::mem::take(&mut *self.collector.events.lock().expect("collector poisoned"));
        let mut out = TickContacts::default();
        let id = |h: RigidBodyHandle| self.body_ids.get(&h).copied();
        for event in raw {
            match event {
                RawContact::Started(h1, h2, p) => {
                    if let (Some(a), Some(b)) = (id(h1), id(h2)) {
                        out.started
                            .push((a, b, nalgebra::Vector3::new(p.x, p.y, p.z)));
                    }
                }
                RawContact::Stopped(h1, h2) => {
                    if let (Some(a), Some(b)) = (id(h1), id(h2)) {
                        out.stopped.push((a, b));
                    }
                }
                RawContact::Force(h1, h2, f) => {
                    if let (Some(a), Some(b)) = (id(h1), id(h2)) {
                        out.forces.push((a, b, f));
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use botrail_physics::{BodyDesc, BodyProps};
    use nalgebra::Vector3;
    use parry3d_f64::shape::SharedShape;

    fn box_world(drop_height: f64, with_floor: bool) -> WorldDesc {
        let mut world = WorldDesc::new();
        if with_floor {
            world.bodies.push(BodyDesc {
                kind: BodyKind::Static,
                pose: Isometry3::translation(0.0, 0.0, -0.05),
                parts: vec![(Pose::identity(), SharedShape::cuboid(2.0, 2.0, 0.05))],
                props: BodyProps::default(),
            });
        }
        world.bodies.push(BodyDesc {
            kind: BodyKind::Dynamic,
            pose: Isometry3::translation(0.0, 0.0, drop_height),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.05, 0.025, 0.015))],
            props: BodyProps {
                mass: Some(0.2),
                ..BodyProps::dynamic()
            },
        });
        world
    }

    /// Gravity/dt plumbing: before any contact, the fall must match the
    /// analytic `½gt²` to within the integrator's first-order error bound
    /// `g·dt·t` (the exact discrete sum depends on rapier's internal
    /// solver substepping — an implementation detail not asserted here).
    /// This catches a wrong g, a wrong dt, or double stepping.
    #[test]
    fn free_fall_matches_analytic_solution() {
        let mut backend = RapierBackend::new();
        backend.reset(&box_world(10.0, false)).unwrap();
        let dt = 1.0 / 400.0;
        let n = 200; // 0.5 s, ~1.2 m of fall — nowhere near anything
        for _ in 0..n {
            backend.step(dt);
        }
        let t = dt * n as f64;
        let dropped = 10.0 - backend.body_pose(BodyId(0)).translation.z;
        let analytic = 0.5 * 9.81 * t * t;
        let bound = 9.81 * dt * t;
        assert!(
            (dropped - analytic).abs() < bound,
            "dropped {dropped}, analytic {analytic}, bound {bound}"
        );
    }

    /// Same world, same steps → bitwise same poses (single-machine
    /// determinism, the physics bake's documented guarantee).
    #[test]
    fn two_runs_are_bitwise_identical() {
        let run = || {
            let mut backend = RapierBackend::new();
            backend.reset(&box_world(1.0, true)).unwrap();
            for _ in 0..1200 {
                backend.step(1.0 / 400.0);
            }
            backend.body_pose(BodyId(1))
        };
        let (a, b) = (run(), run());
        assert_eq!(a.translation.vector, b.translation.vector);
        assert_eq!(a.rotation.coords, b.rotation.coords);
    }

    /// Belt world: a bed slab with its top face at z = 0, a zone whose
    /// lower face sits exactly on that surface (the natural authoring —
    /// [`ZONE_MARGIN`] is what makes the penetrating contact points still
    /// count), and one box resting on it. `belt_first` flips creation
    /// order, which is what decides which side of a contact pair each
    /// body lands on — the tangent-velocity sign convention must survive
    /// both.
    fn belt_world(bed_friction: f64, box_friction: f64, belt_first: bool) -> WorldDesc {
        use botrail_physics::{PhysicsMaterial, SurfaceVelocityZone};
        let mut world = WorldDesc::new();
        let bed = BodyDesc {
            kind: BodyKind::Static,
            pose: Isometry3::translation(0.0, 0.0, -0.05),
            parts: vec![(Pose::identity(), SharedShape::cuboid(2.0, 0.2, 0.05))],
            props: BodyProps {
                material: PhysicsMaterial {
                    friction: bed_friction,
                    ..Default::default()
                },
                ..BodyProps::default()
            },
        };
        let cargo = BodyDesc {
            kind: BodyKind::Dynamic,
            pose: Isometry3::translation(-1.5, 0.0, 0.05),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.05, 0.04, 0.03))],
            props: BodyProps {
                mass: Some(0.3),
                material: PhysicsMaterial {
                    friction: box_friction,
                    ..Default::default()
                },
                ..BodyProps::dynamic()
            },
        };
        if belt_first {
            world.bodies.push(bed);
            world.bodies.push(cargo);
        } else {
            world.bodies.push(cargo);
            world.bodies.push(bed);
        }
        world.zones.push(SurfaceVelocityZone {
            pose: Isometry3::translation(0.0, 0.0, 0.1),
            half_extents: Vector3::new(2.0, 0.2, 0.1),
            velocity: Vector3::new(0.3, 0.0, 0.0),
            active: true,
        });
        world
    }

    fn conveyed_velocity(world: &WorldDesc, cargo: BodyId) -> f64 {
        let mut backend = RapierBackend::new();
        backend.reset(world).unwrap();
        let dt = 1.0 / 400.0;
        for _ in 0..400 {
            backend.step(dt); // 1 s: land, grip, reach belt speed
        }
        let x1 = backend.body_pose(cargo).translation.x;
        for _ in 0..400 {
            backend.step(dt); // second 1 s: steady state
        }
        let x2 = backend.body_pose(cargo).translation.x;
        x2 - x1
    }

    /// A box on an active belt is carried at the belt velocity — in the
    /// velocity's own direction — whichever side of the contact pair the
    /// dynamic body lands on (creation order flips it).
    #[test]
    fn a_belt_conveys_at_its_surface_velocity() {
        for belt_first in [true, false] {
            let world = belt_world(0.6, 0.6, belt_first);
            let cargo = BodyId(if belt_first { 1 } else { 0 });
            let travelled = conveyed_velocity(&world, cargo);
            assert!(
                (travelled - 0.3).abs() < 0.03,
                "belt_first={belt_first}: travelled {travelled} in 1 s, belt is 0.3 m/s"
            );
        }
    }

    /// An inactive zone is just a floor: the box settles where it landed.
    #[test]
    fn an_inactive_belt_is_just_a_floor() {
        let mut world = belt_world(0.6, 0.6, true);
        world.zones[0].active = false;
        let travelled = conveyed_velocity(&world, BodyId(1));
        assert!(travelled.abs() < 1e-6, "moved {travelled} on a stopped belt");
    }

    /// Frictionless contact transmits no belt drive: the surface slides
    /// under the box and the box stays.
    #[test]
    fn zero_friction_slips_on_the_belt() {
        let world = belt_world(0.0, 0.0, true);
        let travelled = conveyed_velocity(&world, BodyId(1));
        assert!(travelled.abs() < 5e-3, "moved {travelled} with zero friction");
    }

    /// Dropped box lands on the floor slab, settles at its half height,
    /// and the engine puts it to sleep.
    #[test]
    fn dropped_box_settles_and_sleeps() {
        let mut backend = RapierBackend::new();
        backend.reset(&box_world(1.0, true)).unwrap();
        for _ in 0..1600 {
            backend.step(1.0 / 400.0);
        }
        let pose = backend.body_pose(BodyId(1));
        assert!(
            (pose.translation.z - 0.015).abs() < 2e-3,
            "settled z = {}",
            pose.translation.z
        );
        assert!(backend.is_sleeping(BodyId(1)), "box should be asleep");
    }
}
