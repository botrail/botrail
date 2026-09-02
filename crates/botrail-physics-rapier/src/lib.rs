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
    BodyId, BodyKind, JointKind, JointMotor, PhysicsBackend, PhysicsError, TickContacts,
    Velocity, WorldDesc, DEFAULT_DENSITY,
};
use nalgebra::Isometry3;
use rapier3d_f64::dynamics::{
    CCDSolver, GenericJoint, ImpulseJointHandle, ImpulseJointSet, IntegrationParameters,
    IslandManager, JointAxesMask, JointAxis, MotorModel, MultibodyJointSet, RigidBodyBuilder,
    RigidBodyHandle, RigidBodySet, RigidBodyType,
};
use rapier3d_f64::geometry::{
    BroadPhaseBvh, ColliderBuilder, ColliderSet, CollisionEvent, ContactPair, Group,
    InteractionGroups, NarrowPhase,
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
        let dynamic1 = ctx.rigid_body1.is_some_and(|h| ctx.bodies[h].is_dynamic());
        let dynamic2 = ctx.rigid_body2.is_some_and(|h| ctx.bodies[h].is_dynamic());
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
        Some((colliders.get(c1)?.parent()?, colliders.get(c2)?.parent()?))
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
        let Some((b1, b2)) = Self::bodies(colliders, event.collider1(), event.collider2()) else {
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
    /// Driven joints in `WorldDesc::joints` order, with what the target
    /// supply and position read-back need.
    joints: Vec<DrivenJoint>,
    hooks: ConveyorHooks,
    collector: ContactCollector,
}

/// One driven joint's runtime bookkeeping.
struct DrivenJoint {
    handle: ImpulseJointHandle,
    parent: RigidBodyHandle,
    child: RigidBodyHandle,
    /// Joint frames on each body, x-axis = the joint axis.
    frame1: Pose,
    frame2: Pose,
    kind: JointKind,
    motor: JointMotor,
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
            joints: Vec::new(),
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
            self.body_ids
                .insert(handle, BodyId(self.handles.len() as u32));
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
                // Bodies sharing a nonzero group never collide with each
                // other: one robot's links don't fight themselves once a
                // finger goes dynamic. Group 0 collides with everything.
                if desc.group != 0 {
                    let own = Group::from_bits_truncate(1 << ((desc.group - 1) % 32));
                    collider = collider.collision_groups(InteractionGroups {
                        memberships: own,
                        filter: !own,
                        test_mode: Default::default(),
                    });
                }
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
        // Driven joints (a friction gripper's fingers): the free axis is
        // +X of the joint frames, everything else locked; the position
        // motor is force-capped and ForceBased — the cap is a force, not
        // a mass-scaled gain (design-grasping.md G3).
        for desc in &world.joints {
            let x = nalgebra::Vector3::x_axis();
            let axis = nalgebra::Unit::new_normalize(desc.axis);
            let align = nalgebra::UnitQuaternion::rotation_between_axis(&x, &axis)
                .unwrap_or_else(|| {
                    nalgebra::UnitQuaternion::from_axis_angle(
                        &nalgebra::Vector3::y_axis(),
                        std::f64::consts::PI,
                    )
                });
            let align = nalgebra::Isometry3::from_parts(nalgebra::Translation3::identity(), align);
            let frame1 = to_pose(&(desc.local1 * align));
            let frame2 = to_pose(&(desc.local2 * align));
            let (mask, axis) = match desc.kind {
                JointKind::Prismatic => (JointAxesMask::LOCKED_PRISMATIC_AXES, Some(JointAxis::LinX)),
                JointKind::Revolute => (JointAxesMask::LOCKED_REVOLUTE_AXES, Some(JointAxis::AngX)),
                // A weld: everything locked, nothing motored.
                JointKind::Fixed => (JointAxesMask::LOCKED_FIXED_AXES, None),
            };
            let mut joint = GenericJoint::new(mask);
            joint.local_frame1 = frame1;
            joint.local_frame2 = frame2;
            joint.set_contacts_enabled(false);
            if let Some(axis) = axis {
                joint.set_motor_model(axis, MotorModel::ForceBased);
                joint.set_motor_position(axis, 0.0, desc.motor.stiffness, desc.motor.damping);
                joint.set_motor_max_force(axis, desc.motor.max_force);
                if let Some((lo, hi)) = desc.limits {
                    joint.set_limits(axis, [lo, hi]);
                }
            }
            let parent = self.handles[desc.parent.0 as usize];
            let child = self.handles[desc.child.0 as usize];
            let handle = self.impulse_joints.insert(parent, child, joint, true);
            self.joints.push(DrivenJoint {
                handle,
                parent,
                child,
                frame1,
                frame2,
                kind: desc.kind,
                motor: desc.motor,
            });
        }
        Ok(())
    }

    fn set_kinematic_pose(&mut self, body: BodyId, pose: Isometry3<f64>) {
        self.body_mut(body)
            .set_next_kinematic_position(to_pose(&pose));
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

    fn set_joint_target(&mut self, joint: usize, position: f64) {
        let dj = &self.joints[joint];
        let axis = match dj.kind {
            JointKind::Prismatic => JointAxis::LinX,
            JointKind::Revolute => JointAxis::AngX,
            JointKind::Fixed => return, // welds take no target
        };
        let (stiffness, damping, cap) = (dj.motor.stiffness, dj.motor.damping, dj.motor.max_force);
        if let Some(j) = self.impulse_joints.get_mut(dj.handle, true) {
            j.data.set_motor_position(axis, position, stiffness, damping);
            j.data.set_motor_max_force(axis, cap);
        }
    }

    fn joint_position(&self, joint: usize) -> f64 {
        let dj = &self.joints[joint];
        // Relative joint transform: frame1⁻¹ ∘ pose1⁻¹ ∘ pose2 ∘ frame2.
        // Its x-translation is the prismatic coordinate; its rotation is
        // (up to constraint drift) about +X, whose angle is the revolute
        // coordinate.
        let p1 = *self.bodies[dj.parent].position();
        let p2 = *self.bodies[dj.child].position();
        let rel = (p1 * dj.frame1).inverse() * (p2 * dj.frame2);
        match dj.kind {
            JointKind::Prismatic => rel.translation.x,
            JointKind::Revolute => {
                let q = rel.rotation;
                2.0 * q.x.atan2(q.w)
            }
            JointKind::Fixed => 0.0,
        }
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
            group: 0,
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
            group: 0,
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
            group: 0,
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
            group: 0,
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
        assert!(
            travelled.abs() < 1e-6,
            "moved {travelled} on a stopped belt"
        );
    }

    /// Frictionless contact transmits no belt drive: the surface slides
    /// under the box and the box stays.
    #[test]
    fn zero_friction_slips_on_the_belt() {
        let world = belt_world(0.0, 0.0, true);
        let travelled = conveyed_velocity(&world, BodyId(1));
        assert!(
            travelled.abs() < 5e-3,
            "moved {travelled} with zero friction"
        );
    }

    /// A parallel grasp, as the rollout stages it: a dynamic part resting
    /// on scenery, two kinematic finger pads converging on it (the same
    /// substep-interpolated pose supply the kinematic mirror uses), closing
    /// to a signed clearance per side — positive = stop short, negative =
    /// overtravel into the part. Returns (finger×part contact episodes
    /// started, part displacement from start, |part velocity| at the end).
    ///
    /// This is the measurement behind `grasp_close`'s default clearance
    /// (design-grasping.md §9): the close must reliably *report contact*
    /// without meaningfully disturbing the part before the attach.
    fn close_fingers(clearance: f64) -> (usize, f64, f64) {
        let part_half = 0.02; // 40 mm wide box
        let pad_half = 0.005; // 10 mm thick pads
        let mut world = WorldDesc::new();
        world.bodies.push(BodyDesc {
            kind: BodyKind::Static,
            pose: Isometry3::translation(0.0, 0.0, -0.05),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.5, 0.5, 0.05))],
            props: BodyProps::default(),
            group: 0,
        });
        world.bodies.push(BodyDesc {
            kind: BodyKind::Dynamic,
            pose: Isometry3::translation(0.0, 0.0, part_half),
            parts: vec![(
                Pose::identity(),
                SharedShape::cuboid(part_half, part_half, part_half),
            )],
            props: BodyProps {
                mass: Some(0.2),
                ..BodyProps::dynamic()
            },
            group: 0,
        });
        let start = 0.05; // pad faces 30 mm out from the part face
        let goal = part_half + pad_half + clearance;
        for side in [-1.0, 1.0] {
            world.bodies.push(BodyDesc {
                kind: BodyKind::Kinematic,
                pose: Isometry3::translation(side * start, 0.0, part_half),
                parts: vec![(
                    Pose::identity(),
                    SharedShape::cuboid(pad_half, 0.015, 0.015),
                )],
                props: BodyProps::default(),
            group: 0,
            });
        }
        let mut backend = RapierBackend::new();
        backend.reset(&world).unwrap();
        let (dt, substeps) = (0.01, 4);
        let mut contacts = 0usize;
        let mut run = |from: f64, to: f64, ticks: usize, contacts: &mut usize| {
            for tick in 0..ticks {
                for sub in 0..substeps {
                    let s = (tick as f64 + (sub + 1) as f64 / substeps as f64) / ticks as f64;
                    let x = from + (to - from) * s;
                    backend.set_kinematic_pose(BodyId(2), Isometry3::translation(-x, 0.0, 0.02));
                    backend.set_kinematic_pose(BodyId(3), Isometry3::translation(x, 0.0, 0.02));
                    backend.step(dt / substeps as f64);
                }
                let drained = backend.drain_contacts();
                *contacts += drained
                    .started
                    .iter()
                    .filter(|(a, b, _)| {
                        let pair = (a.0.min(b.0), a.0.max(b.0));
                        pair == (1, 2) || pair == (1, 3)
                    })
                    .count();
            }
        };
        run(start, goal, 50, &mut contacts); // 0.5 s close
        // Hold closed for 0.5 s — the window between "closed" and "attach"
        // never lasts longer in a real sequence.
        run(goal, goal, 50, &mut contacts);
        let pose = backend.body_pose(BodyId(1));
        let moved = (pose.translation.vector - Vector3::new(0.0, 0.0, part_half)).norm();
        (contacts, moved, 0.0)
    }

    /// Closing to a small *overtravel* must both report finger contact and
    /// leave the part essentially where it stood (the opposing pads
    /// cancel). This pins the default clearance `grasp_close` ships with.
    #[test]
    fn kinematic_finger_squeeze_touches_without_ejecting() {
        let (contacts, moved, _) = close_fingers(-0.0005);
        assert!(contacts >= 2, "expected both pads to report contact, got {contacts}");
        assert!(moved < 2e-3, "squeeze displaced the part {} m", moved);
    }

    /// A close that stops *short* of the surface. Measured (rapier 0.34):
    /// contact events still fire at a 0.2 mm gap (the narrow phase's
    /// prediction distance covers it) and the predictive contacts nudge
    /// the part ~1.3 mm — a hair *more* drift than the symmetric squeeze,
    /// whose opposing pushes cancel. Both support the squeeze default;
    /// this test documents the near-miss behavior so a rapier bump that
    /// changes it is noticed.
    #[test]
    fn kinematic_finger_near_miss_still_reports_contact() {
        let (contacts, moved, _) = close_fingers(0.0002);
        assert!(contacts >= 2, "expected predictive contacts, got {contacts}");
        assert!(moved < 3e-3, "a non-touching close displaced the part {} m", moved);
    }

    /// The friction-grasp mechanism (design-grasping.md G3 spike): a
    /// kinematic palm carries two dynamic finger bodies on prismatic
    /// impulse joints driven by force-capped position motors. The motors
    /// command an overtravel; contact stalls them at the cap, the part is
    /// held by friction alone (no attachment), and the palm lifts.
    /// Returns (part height gain, |part-to-palm drift| during the carry).
    fn friction_lift(
        max_force: f64,
        part_mass: f64,
        lift: f64,
        accel_model: bool,
        stiffness: f64,
        overtravel: f64,
        finger_density: f64,
    ) -> (f64, f64) {
        use rapier3d_f64::dynamics::{
            ImpulseJointSet, MultibodyJointSet, PrismaticJointBuilder, RigidBodyBuilder,
            RigidBodySet,
        };
        use rapier3d_f64::geometry::ColliderBuilder;
        use rapier3d_f64::prelude::MotorModel;

        let mut bodies = RigidBodySet::new();
        let mut colliders = rapier3d_f64::geometry::ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();

        // Floor and a 40 mm part resting on it.
        let floor = bodies.insert(RigidBodyBuilder::fixed().pose(to_pose(
            &Isometry3::translation(0.0, 0.0, -0.05),
        )));
        colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.5, 0.5, 0.05).friction(0.6).build(),
            floor,
            &mut bodies,
        );
        let part = bodies.insert(
            RigidBodyBuilder::dynamic()
                .pose(to_pose(&Isometry3::translation(0.0, 0.0, 0.02)))
                .build(),
        );
        colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.02, 0.02, 0.02)
                .friction(0.6)
                .density(part_mass / 0.04f64.powi(3)),
            part,
            &mut bodies,
        );

        // Kinematic palm above, two dynamic fingers hanging off it on
        // prismatic joints along x.
        let palm_z = 0.09;
        let palm = bodies.insert(
            RigidBodyBuilder::kinematic_position_based()
                .pose(to_pose(&Isometry3::translation(0.0, 0.0, palm_z))),
        );
        colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.06, 0.02, 0.01).friction(0.6),
            palm,
            &mut bodies,
        );
        let mut fingers = Vec::new();
        for side in [-1.0f64, 1.0] {
            let x0 = side * 0.05;
            let finger = bodies.insert(
                RigidBodyBuilder::dynamic()
                    .pose(to_pose(&Isometry3::translation(x0, 0.0, palm_z - 0.045))),
            );
            colliders.insert_with_parent(
                ColliderBuilder::cuboid(0.005, 0.015, 0.035)
                    .friction(0.9)
                    .density(finger_density),
                finger,
                &mut bodies,
            );
            // Joint axis +x on both frames; finger q = displacement from
            // its zero pose. Closing drives toward the centre.
            let mut joint = PrismaticJointBuilder::new(Vector::X).build();
            joint.set_local_anchor1(Vector::new(x0, 0.0, -0.045));
            joint.set_local_anchor2(Vector::ZERO);
            joint.set_contacts_enabled(false);
            joint.set_motor_model(if accel_model {
                MotorModel::AccelerationBased
            } else {
                MotorModel::ForceBased
            });
            // Stiff spring so the error saturates the cap on contact;
            // damping scaled to the cap so the cap-limited FREE speed is
            // 0.1 m/s whatever the cap (the same rule the rollout's drive
            // defaults use). An absolute damping would either crawl a
            // feeble motor into never arriving or let a strong one
            // tunnel.
            joint.set_motor_position(0.0, stiffness, max_force / 0.1);
            joint.set_motor_max_force(max_force);
            joint.set_limits([-0.05, 0.05]);
            let handle = impulse_joints.insert(palm, finger, joint, true);
            fingers.push((handle, -side)); // closing direction of q
        }

        let mut pipeline = PhysicsPipeline::new();
        let mut islands = IslandManager::new();
        let mut broad = BroadPhaseBvh::new();
        let mut narrow = NarrowPhase::new();
        let mut ccd = CCDSolver::new();
        let params = {
            IntegrationParameters {
                dt: 1.0 / 400.0,
                ..Default::default()
            }
        };
        let hooks = ();
        let events = ();
        let step = |bodies: &mut RigidBodySet,
                        colliders: &mut rapier3d_f64::geometry::ColliderSet,
                        impulse_joints: &mut ImpulseJointSet,
                        multibody_joints: &mut MultibodyJointSet,
                        islands: &mut IslandManager,
                        broad: &mut BroadPhaseBvh,
                        narrow: &mut NarrowPhase,
                        ccd: &mut CCDSolver,
                        pipeline: &mut PhysicsPipeline| {
            pipeline.step(
                Vector::new(0.0, 0.0, -9.81),
                &params,
                islands,
                broad,
                narrow,
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                ccd,
                &hooks,
                &events,
            );
        };

        // Close: the target follows the authored ramp one substep at a
        // time (exactly how the rollout feeds it), and the commanded end
        // is grasp_close-derived: the surface plus a small overtravel.
        // The realized clamp is what the CONTACT develops at the standing
        // penetration the overtravel buys (measured law: N scales with
        // penetration through the pair's contact softness — model,
        // stiffness and finger mass barely move it) — so the friction
        // drive needs ~2 mm of overtravel to develop a multi-newton
        // clamp, and the force cap is the *ceiling*, not the resting
        // value. Two failure modes pin the envelope: a sub-mm overtravel
        // realizes well under 1 N and the part slips out of the grip; a
        // DEEP full-close command does not stall at the surface at all —
        // the position motor cancels the contact's bias-limited push-out
        // as a disturbance and parks the finger wherever the command
        // says (measured: 12 mm standing embed).
        for k in 0..400 {
            let s = (k + 1) as f64 / 400.0;
            for (handle, dir) in &fingers {
                let joint = impulse_joints.get_mut(*handle, true).unwrap();
                joint.data.set_motor_position(
                    rapier3d_f64::prelude::JointAxis::LinX,
                    dir * (0.025 + overtravel) * s,
                    stiffness,
                    max_force / 0.1,
                );
            }
            step(
                &mut bodies,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                &mut islands,
                &mut broad,
                &mut narrow,
                &mut ccd,
                &mut pipeline,
            );
        }
        let grip_start = from_pose(bodies[part].position()).translation.z;

        // Lift the palm 1 s at `lift` m/s, then hold 0.5 s.
        let mut drift: f64 = 0.0;
        let part_x0 = from_pose(bodies[part].position()).translation.x;
        for k in 0..600 {
            let t = (k as f64 + 1.0) / 400.0;
            let z = palm_z + lift * t.min(1.0);
            bodies[palm].set_next_kinematic_position(to_pose(&Isometry3::translation(
                0.0, 0.0, z,
            )));
            step(
                &mut bodies,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                &mut islands,
                &mut broad,
                &mut narrow,
                &mut ccd,
                &mut pipeline,
            );
            let p = from_pose(bodies[part].position()).translation;
            drift = drift.max((p.x - part_x0).abs());
        }
        let gained = from_pose(bodies[part].position()).translation.z - grip_start;
        (gained, drift)
    }

    #[test]
    #[ignore]
    fn friction_grid() {
        // The question the grid answers: which (finger mass, overtravel)
        // makes the CAP the differentiator — strong cap holds, feeble cap
        // slips — instead of the standing penetration keying the part in
        // place regardless of force.
        for dens in [3000.0, 1.5e4, 3e4] {
            for over in [0.0005, 0.001, 0.002] {
                let (strong, _) = friction_lift(30.0, 0.2, 0.15, false, 1e5, over, dens);
                let (feeble, _) = friction_lift(0.5, 0.2, 0.15, false, 1e5, over, dens);
                eprintln!(
                    "dens={dens:.1e} (finger {:.0} g) over={:.1}mm -> strong gained={strong:.3} feeble gained={feeble:.3}",
                    dens * 2.1e-5 * 1000.0,
                    over * 1000.0
                );
            }
        }
    }

    /// Enough grip force holds the part through the whole lift with no
    /// perceptible lateral drift — friction alone, no attachment.
    #[test]
    fn force_capped_fingers_hold_a_part_by_friction() {
        let (gained, drift) = friction_lift(30.0, 0.2, 0.15, false, 1e5, 0.002, 3000.0);
        assert!(
            gained > 0.13,
            "part should ride the palm up ~0.15 m, gained {gained}"
        );
        assert!(drift < 0.005, "part drifted {drift} m in the grip");
    }

    /// A grip too weak for the load lets the part slip out during the
    /// lift — the drop is the physics' own answer, not a check.
    #[test]
    fn a_weak_grip_drops_the_part() {
        let (gained, _) = friction_lift(0.4, 0.2, 0.15, false, 1e5, 0.002, 3000.0);
        assert!(
            gained < 0.05,
            "0.4 N on a 200 g part should slip, but it gained {gained}"
        );
    }

    /// The same grip-and-lift as `friction_lift`, but through the
    /// backend's own API — `WorldDesc::joints`, `set_joint_target`,
    /// `joint_position` — the exact path the rollout drives. Guards the
    /// GenericJoint assembly in `reset`.
    /// One hanging revolute link with its COM off the pivot: the motor
    /// at k = 1e3 N*m/rad must hold a 0.2 kg, 5 cm lever against gravity
    /// (~0.1 N*m) to within a fraction of a milliradian. Measures the
    /// REAL stiffness the adapter's ForceBased motors deliver.
    #[test]
    fn revolute_motor_holds_gravity() {
        use botrail_physics::{JointDesc, JointKind, JointMotor};
        let mut world = WorldDesc::new();
        world.bodies.push(BodyDesc {
            kind: BodyKind::Kinematic,
            pose: Isometry3::translation(0.0, 0.0, 0.5),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.02, 0.02, 0.02))],
            props: BodyProps::default(),
            group: 1,
        });
        // Link body: joint frame at its origin, box hanging 5 cm below.
        world.bodies.push(BodyDesc {
            kind: BodyKind::Dynamic,
            pose: Isometry3::translation(0.0, 0.0, 0.45),
            parts: vec![(
                Pose::from_translation(Vector::new(0.0, 0.0, -0.05)),
                SharedShape::cuboid(0.01, 0.01, 0.03),
            )],
            props: BodyProps {
                mass: Some(0.2),
                ..BodyProps::dynamic()
            },
            group: 1,
        });
        world.joints.push(JointDesc {
            parent: BodyId(0),
            child: BodyId(1),
            kind: JointKind::Revolute,
            local1: Isometry3::translation(0.0, 0.0, -0.05),
            local2: Isometry3::identity(),
            axis: Vector3::new(1.0, 0.0, 0.0),
            limits: None,
            motor: JointMotor {
                stiffness: 1e3,
                damping: 250.0,
                max_force: 1000.0,
            },
        });
        let mut backend = RapierBackend::new();
        backend.reset(&world).unwrap();
        // Tip the link so gravity has a lever from the start.
        for k in 0..400 {
            let _ = k;
            backend.set_joint_target(0, 0.0);
            backend.step(0.0025);
        }
        let q = backend.joint_position(0);
        eprintln!("rest angle under gravity: {q:+.5} rad");
        assert!(
            q.abs() < 0.001,
            "k=1e3 must hold a 0.1 N*m gravity lever; rested at {q:+.5} rad"
        );
    }

    /// Minimal 2F-85-shaped chain: kinematic palm — revolute (origin
    /// carries a pi yaw, like the real knuckle joints) — knuckle box —
    /// WELD — finger box — revolute — tip box. Commanded to zero, every
    /// measured joint coordinate must stay at zero: a standing offset
    /// here is a frame bug, not physics.
    #[test]
    fn welded_chain_rests_at_zero() {
        use botrail_physics::{JointDesc, JointKind, JointMotor};
        let mut world = WorldDesc::new();
        // palm (kinematic)
        world.bodies.push(BodyDesc {
            kind: BodyKind::Kinematic,
            pose: Isometry3::translation(0.0, 0.0, 0.5),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.03, 0.03, 0.02))],
            props: BodyProps::default(),
            group: 1,
        });
        let yaw = Isometry3::rotation(Vector3::new(0.0, 0.0, std::f64::consts::PI));
        let knuckle_origin = Isometry3::translation(0.0, -0.03, -0.05) * yaw;
        let weld_origin = Isometry3::translation(0.0, 0.03, -0.004);
        let tip_origin = Isometry3::translation(0.0, 0.006, -0.047);
        let palm = Isometry3::translation(0.0, 0.0, 0.5);
        let knuckle_pose = palm * knuckle_origin;
        let finger_pose = knuckle_pose * weld_origin;
        let tip_pose = finger_pose * tip_origin;
        for pose in [knuckle_pose, finger_pose, tip_pose] {
            world.bodies.push(BodyDesc {
                kind: BodyKind::Dynamic,
                pose,
                // Hang the shape off the joint frame: gravity gets a real
                // lever, like a mesh link's off-pivot COM (a centred box
                // made this test pass vacuously once).
                parts: vec![(
                    Pose::from_translation(Vector::new(0.0, 0.015, -0.03)),
                    SharedShape::cuboid(0.01, 0.01, 0.02),
                )],
                props: BodyProps {
                    mass: Some(0.2),
                    ..BodyProps::dynamic()
                },
                group: 1,
            });
        }
        let motor = JointMotor {
            stiffness: 1e5,
            damping: 250.0,
            max_force: 100.0,
        };
        world.joints.push(JointDesc {
            parent: BodyId(0),
            child: BodyId(1),
            kind: JointKind::Revolute,
            local1: knuckle_origin,
            local2: Isometry3::identity(),
            axis: Vector3::new(1.0, 0.0, 0.0),
            limits: Some((0.0, 0.8)),
            motor,
        });
        world.joints.push(JointDesc {
            parent: BodyId(2),
            child: BodyId(3),
            kind: JointKind::Revolute,
            local1: tip_origin,
            local2: Isometry3::identity(),
            axis: Vector3::new(1.0, 0.0, 0.0),
            limits: None,
            motor,
        });
        world.joints.push(JointDesc {
            parent: BodyId(1),
            child: BodyId(2),
            kind: JointKind::Fixed,
            local1: weld_origin,
            local2: Isometry3::identity(),
            axis: Vector3::new(1.0, 0.0, 0.0),
            limits: None,
            motor: JointMotor {
                stiffness: 0.0,
                damping: 0.0,
                max_force: 0.0,
            },
        });
        let mut backend = RapierBackend::new();
        backend.reset(&world).unwrap();
        for _ in 0..400 {
            backend.set_joint_target(0, 0.0);
            backend.set_joint_target(1, 0.0);
            backend.step(0.0025);
        }
        let q0 = backend.joint_position(0);
        let q1 = backend.joint_position(1);
        eprintln!("rest q0={q0:+.4} q1={q1:+.4}");
        assert!(
            q0.abs() < 0.01 && q1.abs() < 0.01,
            "chain commanded to zero rests at q0={q0:+.4}, q1={q1:+.4}"
        );
    }

    #[test]
    fn worlddesc_joints_grip_and_lift() {
        use botrail_physics::{JointDesc, JointKind, JointMotor};
        let mut world = WorldDesc::new();
        // pedestal (scenery)
        world.bodies.push(BodyDesc {
            kind: BodyKind::Static,
            pose: Isometry3::translation(0.0, 0.0, 0.105),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.06, 0.06, 0.105))],
            props: BodyProps::default(),
            group: 0,
        });
        // part
        world.bodies.push(BodyDesc {
            kind: BodyKind::Dynamic,
            pose: Isometry3::translation(0.0, 0.0, 0.23),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.02, 0.01, 0.02))],
            props: BodyProps {
                mass: Some(0.2),
                material: botrail_physics::PhysicsMaterial {
                    friction: 0.6,
                    ..Default::default()
                },
                ..BodyProps::dynamic()
            },
            group: 0,
        });
        // palm (kinematic, group 1) — high enough that the fingers
        // straddle the part's UPPER half: a finger that reaches below the
        // part's bottom is buried in the pedestal, and dragging through
        // that is what a first cut of this fixture silently measured.
        let grp = 1;
        world.bodies.push(BodyDesc {
            kind: BodyKind::Kinematic,
            pose: Isometry3::translation(0.0, 0.0, 0.315),
            parts: vec![(
                Pose::from_translation(Vector::new(0.0, 0.0, -0.02)),
                SharedShape::cuboid(0.04, 0.03, 0.02),
            )],
            props: BodyProps::default(),
            group: grp,
        });
        // fingers (dynamic, group 1), rubber pads
        for side in [-1.0f64, 1.0] {
            world.bodies.push(BodyDesc {
                kind: BodyKind::Dynamic,
                pose: Isometry3::translation(side * 0.035, 0.0, 0.275),
                parts: vec![(
                    Pose::from_translation(Vector::new(0.0, 0.0, -0.03)),
                    SharedShape::cuboid(0.005, 0.01, 0.03),
                )],
                props: BodyProps {
                    // The rollout's finger mass floor
                    // (`GripperDrive::finger_mass`): contact stiffness is
                    // pair-mass-scaled, so a mesh-derived 12 g finger tops
                    // out near 1.75 N whatever the motor cap — measured
                    // boundary ≈0.05 kg against this 0.2 kg part.
                    mass: Some(0.2),
                    material: botrail_physics::PhysicsMaterial {
                        friction: 0.9,
                        ..Default::default()
                    },
                    ..BodyProps::dynamic()
                },
                group: grp,
            });
        }
        for (i, side) in [-1.0f64, 1.0].into_iter().enumerate() {
            world.joints.push(JointDesc {
                parent: BodyId(2),
                child: BodyId(3 + i as u32),
                kind: JointKind::Prismatic,
                local1: Isometry3::translation(side * 0.035, 0.0, -0.04),
                local2: Isometry3::identity(),
                axis: Vector3::new(1.0, 0.0, 0.0),
                limits: Some((-0.028, 0.028)),
                motor: JointMotor {
                    stiffness: 1e5,
                    damping: 300.0,
                    max_force: 30.0,
                },
            });
        }
        let mut backend = RapierBackend::new();
        backend.reset(&world).unwrap();
        let dt = 0.01 / 4.0;
        // close: ramp targets to a 2 mm overtravel (gap 10 mm each side)
        for k in 0..200 {
            let s = (k + 1) as f64 / 200.0;
            backend.set_joint_target(0, 0.012 * s);
            backend.set_joint_target(1, -0.012 * s);
            for _ in 0..4 {
                backend.step(dt);
            }
        }
        let q0 = backend.joint_position(0);
        for id in [1u32, 3, 4] {
            let p = backend.body_pose(BodyId(id)).translation;
            eprintln!("body {id}: ({:+.4}, {:+.4}, {:+.4})", p.x, p.y, p.z);
        }
        // lift the palm 0.15 m over 1 s, supplied per substep like the
        // rollout's interpolated kinematic feed (a per-tick stair-step
        // gives the palm 4x velocity pulses that shear a marginal grip).
        for k in 0..400 {
            let z = 0.315 + 0.15 * (k + 1) as f64 / 400.0;
            backend.set_kinematic_pose(BodyId(2), Isometry3::translation(0.0, 0.0, z));
            backend.step(dt);
        }
        let part = backend.body_pose(BodyId(1)).translation.z;
        for id in [2u32, 3, 4] {
            let p = backend.body_pose(BodyId(id)).translation;
            eprintln!("after lift body {id}: ({:+.4}, {:+.4}, {:+.4})", p.x, p.y, p.z);
        }
        let contacts = backend.drain_contacts();
        let mut peak = 0.0f64;
        for (a, b, f) in &contacts.forces {
            if matches!((a.0, b.0), (1, 3 | 4) | (3 | 4, 1)) {
                peak = peak.max(*f);
            }
        }
        eprintln!(
            "finger-part force events: {} peak {:.2} N; started pairs: {:?}",
            contacts.forces.len(),
            peak,
            contacts
                .started
                .iter()
                .map(|(a, b, _)| (a.0, b.0))
                .collect::<Vec<_>>()
        );
        assert!(
            contacts
                .started
                .iter()
                .any(|(a, b, _)| { matches!((a.0, b.0), (1, 3 | 4) | (3 | 4, 1)) }),
            "finger-part contacts must register"
        );
        assert!(
            part > 0.33,
            "the part should ride the palm up by friction (q_close was {q0:.4}), ended z = {part:.3}"
        );
    }

    /// The raw `friction_lift` world, replayed byte-for-byte through the
    /// backend API. If this diverges from `friction_lift`'s outcome, the
    /// adapter is the difference.
    #[test]
    fn worlddesc_matches_raw_spike() {
        use botrail_physics::{JointDesc, JointKind, JointMotor};
        let mut world = WorldDesc::new();
        world.bodies.push(BodyDesc {
            kind: BodyKind::Static,
            pose: Isometry3::translation(0.0, 0.0, -0.05),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.5, 0.5, 0.05))],
            props: BodyProps {
                material: botrail_physics::PhysicsMaterial {
                    friction: 0.6,
                    ..Default::default()
                },
                ..BodyProps::default()
            },
            group: 0,
        });
        world.bodies.push(BodyDesc {
            kind: BodyKind::Dynamic,
            pose: Isometry3::translation(0.0, 0.0, 0.02),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.02, 0.02, 0.02))],
            props: BodyProps {
                mass: Some(0.2),
                material: botrail_physics::PhysicsMaterial {
                    friction: 0.6,
                    ..Default::default()
                },
                ..BodyProps::dynamic()
            },
            group: 0,
        });
        world.bodies.push(BodyDesc {
            kind: BodyKind::Kinematic,
            pose: Isometry3::translation(0.0, 0.0, 0.09),
            parts: vec![(Pose::identity(), SharedShape::cuboid(0.06, 0.02, 0.01))],
            props: BodyProps::default(),
            group: 0,
        });
        for side in [-1.0f64, 1.0] {
            world.bodies.push(BodyDesc {
                kind: BodyKind::Dynamic,
                pose: Isometry3::translation(side * 0.05, 0.0, 0.045),
                parts: vec![(Pose::identity(), SharedShape::cuboid(0.005, 0.015, 0.035))],
                props: BodyProps {
                    // The spike's plank-density fingers (63 g). Dropping
                    // this to the shape-derived ~6 g is the single change
                    // that makes the ride fail — the measurement behind
                    // the rollout's finger mass floor.
                    mass: Some(3000.0 * 8.0 * 0.005 * 0.015 * 0.035),
                    material: botrail_physics::PhysicsMaterial {
                        friction: 0.9,
                        ..Default::default()
                    },
                    ..BodyProps::dynamic()
                },
                group: 0,
            });
        }
        for (i, side) in [-1.0f64, 1.0].into_iter().enumerate() {
            world.joints.push(JointDesc {
                parent: BodyId(2),
                child: BodyId(3 + i as u32),
                kind: JointKind::Prismatic,
                local1: Isometry3::translation(side * 0.05, 0.0, -0.045),
                local2: Isometry3::identity(),
                axis: Vector3::new(1.0, 0.0, 0.0),
                limits: Some((-0.05, 0.05)),
                motor: JointMotor {
                    stiffness: 1e5,
                    damping: 300.0,
                    max_force: 30.0,
                },
            });
        }
        let mut backend = RapierBackend::new();
        backend.reset(&world).unwrap();
        let dt = 1.0 / 400.0;
        for k in 0..400 {
            let s = (k + 1) as f64 / 400.0;
            backend.set_joint_target(0, -0.027 * s);
            backend.set_joint_target(0, 0.027 * s);
            backend.set_joint_target(1, -0.027 * s);
            backend.step(dt);
        }
        let grip = backend.body_pose(BodyId(1)).translation.z;
        for k in 0..400 {
            let z = 0.09 + 0.15 * (k + 1) as f64 / 400.0;
            backend.set_kinematic_pose(BodyId(2), Isometry3::translation(0.0, 0.0, z));
            backend.step(dt);
        }
        for _ in 0..200 {
            backend.step(dt);
        }
        let end = backend.body_pose(BodyId(1)).translation.z;
        assert!(
            end - grip > 0.13,
            "the raw spike carries this part 0.15 m; through the adapter it gained {:.3}",
            end - grip
        );
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
