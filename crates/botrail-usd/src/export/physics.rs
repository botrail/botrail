//! UsdPhysics authoring for a *simulation* stage (design-world-physics.md
//! W4-U): the static snapshot a physics engine can own — Isaac Sim / Isaac
//! Lab first — rather than the animation layer the rest of this exporter
//! writes. Three things live here:
//!
//! - **rigid units** ([`UnitSpec`]): one `Xform` per body the engine moves
//!   (dynamic, or kinematic for what a device drives), its member objects
//!   beneath it as colliders and visuals. A conveyor's belt bodies carry
//!   `PhysxSurfaceVelocityAPI`.
//! - **articulations** ([`ArticulationSpec`]): a URDF / catalog / composite
//!   robot as `PhysicsArticulationRootAPI` + rigid-body links (mass,
//!   colliders) + joints (limits, drive, velocity cap, armature, mimic) —
//!   the write side of [`crate::articulation`], which reads it back. A
//!   robot riding a vehicle takes the vehicle's body in as its base, behind
//!   six virtual joints that state — and command — the vehicle's pose
//!   ([`CarrierSpec`]); a second robot on the same vehicle joins that
//!   articulation ([`Ride::Joins`]). A USD-sourced robot rides the same
//!   way, its own stage's anchoring to the world switched off
//!   ([`author_referenced_mount`]).
//! - the **ground** half-space, as a `Plane` collider.
//!
//! PhysX-only vocabulary (`physxJoint:*`, `PhysxMimicJointAPI`,
//! `PhysxArticulationAPI`, `PhysxSurfaceVelocityAPI`) is authored by name:
//! stock USD carries unknown applied schemas untouched, and the attribute
//! names are the ones Isaac Sim 5's PhysxSchema declares.

use super::*;
use botrail_model::{JointType, Link};
use nalgebra::{Matrix3, SymmetricEigen};

/// Who moves a body of the simulation stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyRole {
    /// Moved from outside the solver (a vehicle's chassis, a lift's car, a
    /// belt): `physics:kinematicEnabled`.
    Kinematic,
    /// The engine's.
    Dynamic,
    /// A link of the articulation of the robot that rides it (see
    /// [`CarrierSpec`]): a body the solver moves, through that robot's
    /// virtual base joints.
    Link,
}

/// A conveyor belt, as the body that carries it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeltSpec {
    /// Belt surface velocity, world frame (m/s).
    pub velocity: [f64; 3],
    pub running: bool,
}

/// One rigid body of the simulation stage: an `Xform` at `pose` under
/// `/World/Env`, named like an obstacle (`/`-segments nest). Objects join
/// it through [`ObjectSpec::unit`].
#[derive(Debug, Clone, PartialEq)]
pub struct UnitSpec {
    pub name: String,
    pub role: BodyRole,
    /// World pose of the body frame.
    pub pose: Isometry3<f64>,
    /// Mass (kg) of a dynamic body; `None` leaves it to the consumer's
    /// density default over the colliders.
    pub mass: Option<f64>,
    pub belt: Option<BeltSpec>,
}

/// What a joint's drive may spend, when the scene declared more than the
/// model's limits say.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ServoSpec {
    /// Force cap (N·m, N).
    pub max_force: Option<f64>,
    /// Rated speed (rad/s, m/s).
    pub max_velocity: Option<f64>,
    /// Reflected drive inertia (kg·m², kg).
    pub armature: Option<f64>,
}

/// A robot authored as a UsdPhysics articulation.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ArticulationSpec {
    /// A free base (a walker, an aircraft): no joint to the world.
    pub floating: bool,
    /// Powered joints hold the exported pose through a position drive;
    /// unpowered ones keep only `passive_damping`.
    pub powered: bool,
    /// Viscous drag of an unpowered joint (N·m·s/rad, N·s/m).
    pub passive_damping: f64,
    /// One entry per `model.actuated_joints`; empty = the model's limits.
    pub servos: Vec<ServoSpec>,
}

/// The vehicle a robot rides, as the base of that robot's articulation.
///
/// A vehicle goes where its program says — botrail's vehicles are
/// commanded, not pushed — and the stage says the same in the one form a
/// physics engine takes on every pipeline: the vehicle's body is a link of
/// the robot's articulation, reached from the world through six virtual
/// joints (`base_x`, `base_y`, `base_z`, `base_yaw`, `base_pitch`,
/// `base_roll`) whose positions *are* the vehicle's pose. Drive them and
/// the vehicle moves, the robot with it: along a floor, up a lift, nose-up
/// on a ramp. It is how Isaac Lab's own mobile manipulators are built
/// (three such joints, for a floor).
///
/// What this replaced, and why (both measured on Isaac Sim 5.1): bolted to
/// the world, the robot stayed behind when its vehicle was posed; bolted
/// to the vehicle's *kinematic* body through a joint outside the
/// articulation, it followed under CPU dynamics only, came up as a
/// floating base, and diverged when its root pose was written on its own.
#[derive(Debug, Clone, PartialEq)]
pub struct CarrierSpec {
    /// Index into [`SimulationSpec::units`] of the vehicle's body — a
    /// [`BodyRole::Link`] unit the robot's base is then bolted to. `None`
    /// when the vehicle has no body of its own, the robot *being* the
    /// machine (a wheeled humanoid modelled whole): the virtual joints
    /// land on the robot's root link.
    pub unit: Option<usize>,
    /// The vehicle's own frame in the world: what its path and stations
    /// speak, and what the virtual joints state (a body's frame is
    /// wherever its first obstacle was authored; this is the machine's).
    pub frame: Isometry3<f64>,
}

/// How a robot rides a vehicle, when it does (one per robot in
/// [`SimulationSpec::rides`]).
#[derive(Debug, Clone, PartialEq)]
pub enum Ride {
    /// The vehicle is this robot's base: the six virtual joints and the
    /// vehicle's body join this robot's articulation.
    Base(CarrierSpec),
    /// The vehicle is already the base of robot `host`'s articulation
    /// (an earlier robot, riding as [`Ride::Base`]): this robot is bolted
    /// to it and joins that articulation — two arms on one cart are one
    /// machine to the engine, addressed through the host. What a machine
    /// of several robots then lacks is collision *between* them: an
    /// articulation's self-collision is off as a whole.
    Joins { host: usize },
}

/// A [`Ride`] resolved to prims and poses: the body a chain ends on, or the
/// body a joining robot is bolted to.
#[derive(Clone, Copy)]
pub(super) enum Mount<'a> {
    /// Hosts the chain from the world. `body` is the vehicle's, or `None`
    /// when the robot is the machine and the chain lands on its root link.
    Base {
        frame: &'a Isometry3<f64>,
        body: Option<(&'a str, &'a Isometry3<f64>)>,
    },
    /// Bolted to `body` — a vehicle's, or the host's root link — and part
    /// of the host's articulation.
    Joins { body: (&'a str, &'a Isometry3<f64>) },
}

/// An articulation to author: its declaration, how it rides, and the
/// prefix its prim names take when it shares an articulation (a robot's
/// joint and link names are its own to the model, and two arms of one
/// model on one cart would clash — PhysX would rename the second's
/// `shoulder_pan` to `shoulder_pan_0`).
#[derive(Clone, Copy)]
pub(super) struct Rooted<'a> {
    pub spec: &'a ArticulationSpec,
    pub mount: Option<Mount<'a>>,
    pub prefix: Option<&'a str>,
}

/// Position-drive gains of a powered joint, SI. The consumer is expected
/// to bring its own (Isaac Lab's actuator configs overwrite these); what
/// is authored only has to hold the pose when the stage is played as it
/// stands. The angular pair is the one NVIDIA's own UR10 asset ships.
const DRIVE_STIFFNESS_ANGULAR: f64 = 1.0e5;
const DRIVE_DAMPING_ANGULAR: f64 = 1.0e4;
const DRIVE_STIFFNESS_LINEAR: f64 = 1.0e6;
const DRIVE_DAMPING_LINEAR: f64 = 1.0e5;

/// Mass of a link that states none and has no collider to derive one
/// from — a frame link (`tool0`, a TCP). It still has to be a body for a
/// consumer to address it, and a gram rides a fixed joint unnoticed.
const FRAME_LINK_MASS: f64 = 1.0e-3;
const FRAME_LINK_INERTIA: f32 = 1.0e-6;

/// A prim name under a robot's prefix, when it has one.
pub(super) fn prefixed(prefix: Option<&str>, name: &str) -> String {
    match prefix {
        Some(p) => format!("{p}_{name}"),
        None => name.to_string(),
    }
}

/// The links between a carrier's virtual joints: bodies only because a
/// joint needs two, light next to any vehicle.
const CARRIER_LINK_MASS: f64 = 0.5;
const CARRIER_LINK_INERTIA: f32 = 0.01;
/// Position drives of the virtual joints, SI (N/m and N·m/rad; N·s/m and
/// N·m·s/rad): stiff, and never unpowered — they stand for a machine that
/// is where its controller put it. A 150 kg vehicle sags 0.15 mm on the
/// vertical one.
const CARRIER_STIFFNESS: f64 = 1.0e7;
const CARRIER_DAMPING: f64 = 1.0e5;

fn api_schemas(layer: &mut LayerBuilder, prim: &str, schemas: &[&str]) {
    layer.prim_field(
        prim,
        FieldKey::ApiSchemas,
        Value::TokenListOp(ListOp::prepended(
            schemas
                .iter()
                .map(|s| tf::Token::from(*s))
                .collect::<Vec<_>>(),
        )),
    );
}

/// [`api_schemas`] on a prim that may already apply some (a visual gprim
/// bound to a material).
fn add_api_schemas(layer: &mut LayerBuilder, prim: &str, schemas: &[&str]) {
    let mut all = layer.applied_schemas(prim);
    for schema in schemas {
        if !all.iter().any(|s| s == schema) {
            all.push((*schema).to_string());
        }
    }
    let all: Vec<&str> = all.iter().map(String::as_str).collect();
    api_schemas(layer, prim, &all);
}

fn float(layer: &mut LayerBuilder, prim: &str, name: &str, value: f64) {
    layer.attr(
        prim,
        name,
        "float",
        AttrValue::Default(Value::Float(value as f32)),
    );
}

fn boolean(layer: &mut LayerBuilder, prim: &str, name: &str, value: bool) {
    layer.attr(prim, name, "bool", AttrValue::Default(Value::Bool(value)));
}

fn token(layer: &mut LayerBuilder, prim: &str, name: &str, value: &str) {
    layer.attr(
        prim,
        name,
        "token",
        AttrValue::Uniform(Value::Token(tf::Token::from(value))),
    );
}

fn vec3f(v: &Vector3<f64>) -> Value {
    Value::Vec3f(gf::vec3f(v.x as f32, v.y as f32, v.z as f32))
}

fn quatf(q: &UnitQuaternion<f64>) -> Value {
    Value::Quatf(gf::quatf(q.w as f32, q.i as f32, q.j as f32, q.k as f32))
}

/// Marks a gprim as a collider nobody renders.
pub(super) fn guide_purpose(layer: &mut LayerBuilder, prim: &str) {
    token(layer, prim, "purpose", "guide");
}

/// The body APIs of a rigid unit on its `Xform`.
pub(super) fn author_unit_body(layer: &mut LayerBuilder, prim: &str, unit: &UnitSpec) {
    let mut schemas = vec!["PhysicsRigidBodyAPI"];
    let mass = unit.mass.filter(|_| unit.role != BodyRole::Kinematic);
    if mass.is_some() {
        schemas.push("PhysicsMassAPI");
    }
    if unit.belt.is_some() {
        schemas.push("PhysxSurfaceVelocityAPI");
    }
    api_schemas(layer, prim, &schemas);
    if unit.role == BodyRole::Kinematic {
        boolean(layer, prim, "physics:kinematicEnabled", true);
    }
    if let Some(mass) = mass {
        float(layer, prim, "physics:mass", mass);
    }
    if let Some(belt) = unit.belt {
        // Body-local: the belt keeps carrying along itself wherever the
        // consumer puts the body (a cloned environment, a re-posed line).
        let local = unit.pose.rotation.inverse()
            * Vector3::new(belt.velocity[0], belt.velocity[1], belt.velocity[2]);
        layer.attr(
            prim,
            "physxSurfaceVelocity:surfaceVelocity",
            "vector3f",
            AttrValue::Default(vec3f(&local)),
        );
        boolean(
            layer,
            prim,
            "physxSurfaceVelocity:surfaceVelocityEnabled",
            belt.running,
        );
    }
}

/// The ground half-space `z = height`: an infinite `Plane` collider,
/// guide-purposed (a consumer that wants a floor to look at brings one).
pub(super) fn author_ground(layer: &mut LayerBuilder, height: f64) {
    let prim = "/World/Ground";
    layer.ensure_prim(prim, Specifier::Def, Some("Plane"));
    api_schemas(layer, prim, &["PhysicsCollisionAPI"]);
    token(layer, prim, "axis", "Z");
    guide_purpose(layer, prim);
    layer.xform(
        prim,
        &XformValue::Static(Isometry3::translation(0.0, 0.0, height)),
        None,
    );
}

/// The joint-frame rotation and `physics:axis` token that express `axis`
/// (in the child link's frame): a positive basis vector is its own token
/// under an identity frame — so a re-import lands on the same link frames
/// — and anything else turns the joint frame's X onto it.
fn joint_frame(axis: &Vector3<f64>) -> (UnitQuaternion<f64>, &'static str) {
    const TOL: f64 = 1e-9;
    for (basis, name) in [
        (Vector3::x(), "X"),
        (Vector3::y(), "Y"),
        (Vector3::z(), "Z"),
    ] {
        if (axis - basis).norm() < TOL {
            return (UnitQuaternion::identity(), name);
        }
    }
    let rotation = UnitQuaternion::rotation_between(&Vector3::x(), axis).unwrap_or_else(|| {
        // Antiparallel to X: any half turn about a perpendicular.
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI)
    });
    (rotation, "X")
}

/// `PhysxMimicJointAPI` instance of a joint's single degree of freedom.
fn dof_instance(joint_type: JointType, axis_token: &str) -> String {
    let kind = match joint_type {
        JointType::Prismatic => "trans",
        _ => "rot",
    };
    format!("{kind}{axis_token}")
}

fn author_link_mass(layer: &mut LayerBuilder, prim: &str, link: &Link) {
    let Some(inertial) = link.inertial.as_ref().filter(|i| i.mass > 0.0) else {
        if link.collisions.is_empty() && link.visuals.is_empty() {
            float(layer, prim, "physics:mass", FRAME_LINK_MASS);
            layer.attr(
                prim,
                "physics:diagonalInertia",
                "float3",
                AttrValue::Default(Value::Vec3f(gf::vec3f(
                    FRAME_LINK_INERTIA,
                    FRAME_LINK_INERTIA,
                    FRAME_LINK_INERTIA,
                ))),
            );
            // The two are stated together or not at all (UsdPhysics).
            layer.attr(
                prim,
                "physics:principalAxes",
                "quatf",
                AttrValue::Default(quatf(&UnitQuaternion::identity())),
            );
        }
        // Colliders and no stated mass: the consumer's density default,
        // which in PhysX is botrail's own 1000 kg/m³.
        return;
    };
    float(layer, prim, "physics:mass", inertial.mass);
    layer.attr(
        prim,
        "physics:centerOfMass",
        "point3f",
        AttrValue::Default(vec3f(&inertial.origin.translation.vector)),
    );
    // UsdPhysics states a tensor as its principal moments plus the frame
    // they are taken in; URDF states the full tensor in the inertial
    // frame. Diagonalize, and fold the inertial frame's rotation in.
    let tensor: Matrix3<f64> = (inertial.inertia + inertial.inertia.transpose()) * 0.5;
    if tensor.iter().all(|v| v.abs() < 1e-18) {
        return; // no tensor stated: derived from the colliders at this mass
    }
    let eigen = SymmetricEigen::new(tensor);
    let mut axes = eigen.eigenvectors;
    if axes.determinant() < 0.0 {
        let flipped = -axes.column(2);
        axes.set_column(2, &flipped);
    }
    let principal = inertial.origin.rotation
        * UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(axes));
    layer.attr(
        prim,
        "physics:diagonalInertia",
        "float3",
        AttrValue::Default(vec3f(&eigen.eigenvalues)),
    );
    layer.attr(
        prim,
        "physics:principalAxes",
        "quatf",
        AttrValue::Default(quatf(&principal)),
    );
}

/// Authors the physics of a robot whose link `Xform`s (`link_prims`, in
/// model link order, already posed and carrying their visuals) stand flat
/// under `robot_prim`: every link a rigid body with its colliders, every
/// joint a UsdPhysics joint under `<robot_prim>/joints`. `used` is the
/// robot prim's child names so far.
#[allow(clippy::too_many_arguments)]
pub(super) fn author_articulation(
    layer: &mut LayerBuilder,
    robot: &RobotAnimation,
    rooted: Rooted,
    robot_prim: &str,
    link_prims: &[String],
    used: &mut HashMap<String, usize>,
    meshes: &mut super::meshes::MeshLayers,
    warnings: &mut Vec<String>,
) -> Result<(), UsdExportError> {
    let model = robot.model;
    let spec = rooted.spec;
    let mount = rooted.mount.filter(|_| !spec.floating);
    let joins = matches!(mount, Some(Mount::Joins { .. }));

    for (link, prim) in model.links.iter().zip(link_prims) {
        api_schemas(layer, prim, &["PhysicsRigidBodyAPI", "PhysicsMassAPI"]);
        author_link_mass(layer, prim, link);
        // A link that declares no collision geometry collides as it
        // looks — botrail's own rule (`RobotCollider::from_model`).
        let mut colliders: Vec<String> = Vec::new();
        if link.collisions.is_empty() {
            colliders.extend((0..link.visuals.len()).map(|k| format!("{prim}/Visual_{k}")));
        }
        for (k, shape) in link.collisions.iter().enumerate() {
            let collider = format!("{prim}/Collision_{k}");
            let leaf = prim.rsplit('/').next().unwrap_or(prim);
            author_geometry(
                layer,
                &collider,
                &shape.geometry,
                &XformValue::Static(shape.origin),
                None,
                &format!("{leaf}_c{k}"),
                meshes,
                warnings,
            )?;
            guide_purpose(layer, &collider);
            colliders.push(collider);
        }
        for collider in &colliders {
            // A hull per mesh: what a link needs to collide as, and the
            // only mesh collider PhysX takes on a body it moves besides a
            // decomposition.
            if layer.is_mesh(collider) {
                add_api_schemas(
                    layer,
                    collider,
                    &["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"],
                );
                token(layer, collider, "physics:approximation", "convexHull");
            } else {
                add_api_schemas(layer, collider, &["PhysicsCollisionAPI"]);
            }
        }
    }

    // Where the articulation is rooted. Bolted down — to the world, or to
    // the vehicle that is then its base — the root joint is part of it and
    // the robot prim above both says so. A floating base has to name its
    // root link itself: left to choose, PhysX roots the tree at its most
    // central link (measured in Isaac Sim 5.1 — a six-axis arm with a free
    // base came up rooted at its forearm, its joints reordered). A robot
    // joining another's articulation roots nothing: it is part of the
    // host's tree by its mount joint (membership is by connection, not by
    // prim hierarchy — measured).
    if !joins {
        let root_prim = if spec.floating {
            link_prims[model.root_link].as_str()
        } else {
            robot_prim
        };
        add_api_schemas(
            layer,
            root_prim,
            &["PhysicsArticulationRootAPI", "PhysxArticulationAPI"],
        );
        // One robot never collides with itself in botrail (its allowed-
        // collision matrix is a planning matter); PhysX defaults the other
        // way and neighbouring link hulls overlap. The vehicle a robot
        // rides is a link of the same articulation, so the rider never
        // collides with what carries it either — the bake's rule (the
        // vehicle's body joins the robot's collision group: a massing
        // footprint is drawn *around* the machine standing in it).
        boolean(
            layer,
            root_prim,
            "physxArticulation:enabledSelfCollisions",
            false,
        );
    }

    let scope = format!("{robot_prim}/{}", unique_child(used, "joints"));
    layer.ensure_prim(&scope, Specifier::Def, Some("Scope"));
    let q = robot.joint_samples.and_then(|samples| samples.first());

    // The model's own joints keep their names; what is added takes what
    // is left.
    let mut used_joints = HashMap::new();
    used_joints.insert("root_joint".to_string(), 1usize);
    let joint_prims: Vec<String> = model
        .joints
        .iter()
        .map(|j| {
            format!(
                "{scope}/{}",
                unique_child(
                    &mut used_joints,
                    &prefixed(rooted.prefix, &sanitize_name(&j.name))
                )
            )
        })
        .collect();

    if !spec.floating {
        let root_link = &link_prims[model.root_link];
        let base = robot.link_poses[0][model.root_link];
        let root_body = (root_link.as_str(), &base);
        match mount {
            // Bolted to the world where it stands.
            None => {
                let root_joint = format!("{scope}/root_joint");
                layer.ensure_prim(&root_joint, Specifier::Def, Some("PhysicsFixedJoint"));
                layer.rel(&root_joint, "physics:body1", root_link);
                author_joint_frames(layer, &root_joint, &base, &Isometry3::identity());
            }
            Some(mount) => author_mount(
                layer,
                robot_prim,
                &scope,
                used,
                &mut used_joints,
                rooted.prefix,
                root_body,
                mount,
            ),
        }
    }

    let frames: Vec<(UnitQuaternion<f64>, &'static str)> = model
        .joints
        .iter()
        .map(|j| match j.joint_type {
            JointType::Fixed => (UnitQuaternion::identity(), "X"),
            _ => joint_frame(&j.axis.into_inner()),
        })
        .collect();

    for (ji, joint) in model.joints.iter().enumerate() {
        let prim = &joint_prims[ji];
        let (type_name, angular) = match joint.joint_type {
            JointType::Revolute | JointType::Continuous => ("PhysicsRevoluteJoint", true),
            JointType::Prismatic => ("PhysicsPrismaticJoint", false),
            JointType::Fixed => ("PhysicsFixedJoint", true),
        };
        layer.ensure_prim(prim, Specifier::Def, Some(type_name));
        layer.rel(prim, "physics:body0", &link_prims[joint.parent_link]);
        layer.rel(prim, "physics:body1", &link_prims[joint.child_link]);
        let (frame, axis_token) = frames[ji];
        let rotation = Isometry3::from_parts(Translation3::identity(), frame);
        author_joint_frames(layer, prim, &(joint.origin * rotation), &rotation);
        if joint.joint_type == JointType::Fixed {
            continue;
        }
        token(layer, prim, "physics:axis", axis_token);
        // UsdPhysics speaks degrees on angular dofs, stage units (meters
        // here) on linear ones.
        let unit = if angular {
            180.0 / std::f64::consts::PI
        } else {
            1.0
        };
        if let (Some(limits), true) = (joint.limits, joint.joint_type != JointType::Continuous) {
            float(layer, prim, "physics:lowerLimit", limits.lower * unit);
            float(layer, prim, "physics:upperLimit", limits.upper * unit);
        }

        let mut schemas: Vec<String> = Vec::new();
        // Where the joint stands. PhysX does not read an articulation's
        // configuration off its link transforms: without a stated joint
        // position it starts the machine at zero, whatever pose the links
        // were authored in (measured in Isaac Sim 5.1 — a UR5e exported
        // folded came up straight). A mimic follower states its derived
        // value, so the coupling starts satisfied.
        if let Some(q) = q {
            let state = if angular { "angular" } else { "linear" };
            schemas.push(format!("PhysicsJointStateAPI:{state}"));
            let ns = format!("state:{state}:physics");
            float(
                layer,
                prim,
                &format!("{ns}:position"),
                model.joint_value(ji, q) * unit,
            );
            float(layer, prim, &format!("{ns}:velocity"), 0.0);
        }
        let servo = joint
            .q_index
            .and_then(|_| model.actuated_joints.iter().position(|&a| a == ji))
            .and_then(|k| spec.servos.get(k).copied())
            .unwrap_or_default();
        let max_velocity = servo
            .max_velocity
            .or(joint.limits.map(|l| l.velocity))
            .filter(|v| v.is_finite() && *v > 0.0);
        if max_velocity.is_some() || servo.armature.is_some() {
            schemas.push("PhysxJointAPI".into());
        }
        if let Some(v) = max_velocity {
            float(layer, prim, "physxJoint:maxJointVelocity", v * unit);
        }
        if let Some(a) = servo.armature.filter(|a| a.is_finite() && *a > 0.0) {
            float(layer, prim, "physxJoint:armature", a);
        }

        if let Some(mimic) = joint.mimic {
            // PhysX couples the pair as `q + G·q_ref + γ = 0` in USD units
            // — URDF's `q = m·q_ref + o` with both signs turned. The pair
            // moves the same way (the model refuses anything else), so the
            // gearing is unitless and only the offset converts.
            let instance = dof_instance(joint.joint_type, axis_token);
            let source = &model.joints[mimic.source_joint];
            let ns = format!("physxMimicJoint:{instance}");
            schemas.push(format!("PhysxMimicJointAPI:{instance}"));
            float(layer, prim, &format!("{ns}:gearing"), -mimic.multiplier);
            float(layer, prim, &format!("{ns}:offset"), -mimic.offset * unit);
            layer.rel(
                prim,
                &format!("{ns}:referenceJoint"),
                &joint_prims[mimic.source_joint],
            );
            token(
                layer,
                prim,
                &format!("{ns}:referenceJointAxis"),
                &dof_instance(source.joint_type, frames[mimic.source_joint].1),
            );
        } else if joint.q_index.is_some() {
            let drive = if angular { "angular" } else { "linear" };
            let ns = format!("drive:{drive}:physics");
            schemas.push(format!("PhysicsDriveAPI:{drive}"));
            token(layer, prim, &format!("{ns}:type"), "force");
            let max_force = servo
                .max_force
                .or(joint.limits.map(|l| l.effort))
                .filter(|f| f.is_finite() && *f > 0.0);
            if let Some(f) = max_force {
                float(layer, prim, &format!("{ns}:maxForce"), f);
            }
            // Gains are per USD unit of error (N·m per degree).
            let (stiffness, damping) = match (spec.powered, angular) {
                (true, true) => (DRIVE_STIFFNESS_ANGULAR, DRIVE_DAMPING_ANGULAR),
                (true, false) => (DRIVE_STIFFNESS_LINEAR, DRIVE_DAMPING_LINEAR),
                (false, _) => (0.0, spec.passive_damping),
            };
            float(layer, prim, &format!("{ns}:stiffness"), stiffness / unit);
            float(layer, prim, &format!("{ns}:damping"), damping / unit);
            if let Some(q) = q {
                let target = model.joint_value(ji, q) * unit;
                float(layer, prim, &format!("{ns}:targetPosition"), target);
            }
        }
        if !schemas.is_empty() {
            let schemas: Vec<&str> = schemas.iter().map(String::as_str).collect();
            api_schemas(layer, prim, &schemas);
        }
    }
    Ok(())
}

/// Bolts a robot's root body to what it rides: hosting the chain from the
/// world ([`Mount::Base`] — the chain ends on the vehicle's body and a
/// mount joint bolts the root body to that, or, the robot being the
/// machine, the chain ends on the root body itself), or joining another
/// robot's articulation through a mount joint alone ([`Mount::Joins`]).
#[allow(clippy::too_many_arguments)]
fn author_mount(
    layer: &mut LayerBuilder,
    robot_prim: &str,
    scope: &str,
    used_links: &mut HashMap<String, usize>,
    used_joints: &mut HashMap<String, usize>,
    prefix: Option<&str>,
    root_body: (&str, &Isometry3<f64>),
    mount: Mount,
) {
    let bolted_to = match mount {
        Mount::Base { frame, body } => {
            author_carrier_chain(
                layer,
                robot_prim,
                scope,
                used_links,
                used_joints,
                (frame, body.unwrap_or(root_body)),
            );
            body
        }
        Mount::Joins { body } => Some(body),
    };
    if let Some((body, pose)) = bolted_to {
        let joint = format!(
            "{scope}/{}",
            unique_child(used_joints, &prefixed(prefix, "mount_joint"))
        );
        layer.ensure_prim(&joint, Specifier::Def, Some("PhysicsFixedJoint"));
        layer.rel(&joint, "physics:body0", body);
        layer.rel(&joint, "physics:body1", root_body.0);
        author_joint_frames(
            layer,
            &joint,
            &(pose.inverse() * root_body.1),
            &Isometry3::identity(),
        );
    }
}

/// A USD-sourced robot riding a vehicle. Its stage brings its own physics,
/// anchoring to the world included: every joint the asset ties to the world
/// (`world_joints`, relative to the robot prim) is deactivated by an
/// `over` (`active = false` — the prim is then not on the stage at all;
/// merely disabling the joint left PhysX creating it and warning that its
/// frames had come apart), and the rest is [`author_mount`] — the chain under
/// `chain_parent` (the robot prim itself when the stage composes at
/// identity, a sibling otherwise, so the chain's world-frame poses are not
/// dragged through a unit correction), the mount joint on the root body
/// prim. A joining robot also loses the articulation root its asset
/// applies, or PhysX would find two roots in one tree.
#[allow(clippy::too_many_arguments)]
pub(super) fn author_referenced_mount(
    layer: &mut LayerBuilder,
    robot_prim: &str,
    chain_parent: &str,
    used: &mut HashMap<String, usize>,
    world_joints: &[String],
    root_body: (&str, &Isometry3<f64>),
    mount: Mount,
) {
    for rel in world_joints {
        let prim = format!("{robot_prim}/{rel}");
        layer.ensure_prim(&prim, Specifier::Over, None);
        layer.prim_field(&prim, FieldKey::Active, Value::Bool(false));
    }
    if matches!(mount, Mount::Joins { .. }) {
        layer.prim_field(
            robot_prim,
            FieldKey::ApiSchemas,
            Value::TokenListOp(ListOp {
                deleted_items: vec![tf::Token::from("PhysicsArticulationRootAPI")],
                ..Default::default()
            }),
        );
    }
    if chain_parent != robot_prim {
        layer.ensure_prim(chain_parent, Specifier::Def, Some("Xform"));
    }
    let scope = format!("{chain_parent}/{}", unique_child(used, "joints"));
    layer.ensure_prim(&scope, Specifier::Def, Some("Scope"));
    let mut used_joints = HashMap::new();
    author_mount(
        layer,
        chain_parent,
        &scope,
        used,
        &mut used_joints,
        None,
        root_body,
        mount,
    );
}

/// The world end of a riding robot's articulation (see [`CarrierSpec`]):
/// `root_joint` bolts an anchor link to the world at the origin, and six
/// virtual joints — three slides, then yaw, pitch, roll — carry on from it
/// to `end`, the prim and world pose of the body they carry. Their
/// positions are the vehicle `frame`'s pose
/// (`T(x, y, z) · Rz(yaw) · Ry(pitch) · Rx(roll)`), stated and held.
///
/// The anchor is not decoration: PhysX takes whatever joint reaches the
/// world as the fixed base, its type ignored — slid straight off the
/// world, `base_x` was no joint at all (measured: it was missing from the
/// articulation's joints).
fn author_carrier_chain(
    layer: &mut LayerBuilder,
    robot_prim: &str,
    scope: &str,
    used_links: &mut HashMap<String, usize>,
    used_joints: &mut HashMap<String, usize>,
    (frame, end): (&Isometry3<f64>, (&str, &Isometry3<f64>)),
) {
    let mut virtual_link = |layer: &mut LayerBuilder, name: &str, pose: &Isometry3<f64>| {
        let prim = format!("{robot_prim}/{}", unique_child(used_links, name));
        layer.ensure_prim(&prim, Specifier::Def, Some("Xform"));
        layer.xform(&prim, &XformValue::Static(*pose), None);
        api_schemas(layer, &prim, &["PhysicsRigidBodyAPI", "PhysicsMassAPI"]);
        float(layer, &prim, "physics:mass", CARRIER_LINK_MASS);
        layer.attr(
            &prim,
            "physics:diagonalInertia",
            "float3",
            AttrValue::Default(Value::Vec3f(gf::vec3f(
                CARRIER_LINK_INERTIA,
                CARRIER_LINK_INERTIA,
                CARRIER_LINK_INERTIA,
            ))),
        );
        layer.attr(
            &prim,
            "physics:principalAxes",
            "quatf",
            AttrValue::Default(quatf(&UnitQuaternion::identity())),
        );
        prim
    };

    let identity = Isometry3::identity();
    let mut parent = virtual_link(layer, "carrier_anchor", &identity);
    let root_joint = format!("{scope}/root_joint");
    layer.ensure_prim(&root_joint, Specifier::Def, Some("PhysicsFixedJoint"));
    layer.rel(&root_joint, "physics:body1", &parent);
    author_joint_frames(layer, &root_joint, &identity, &identity);

    let t = frame.translation.vector;
    let (roll, pitch, yaw) = frame.rotation.euler_angles();
    let turn = |axis: Vector3<f64>, angle: f64| {
        Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_scaled_axis(axis * angle),
        )
    };
    let dofs = [
        ("x", "X", t.x, Isometry3::translation(t.x, 0.0, 0.0)),
        ("y", "Y", t.y, Isometry3::translation(0.0, t.y, 0.0)),
        ("z", "Z", t.z, Isometry3::translation(0.0, 0.0, t.z)),
        ("yaw", "Z", yaw, turn(Vector3::z(), yaw)),
        ("pitch", "Y", pitch, turn(Vector3::y(), pitch)),
        ("roll", "X", roll, turn(Vector3::x(), roll)),
    ];
    let mut pose = identity;
    for (k, (name, axis, value, motion)) in dofs.iter().enumerate() {
        let angular = k >= 3;
        pose *= motion;
        // The last joint lands on the carried body, at the vehicle's
        // frame as that body sees it.
        let (child, local1) = if k + 1 == dofs.len() {
            (end.0.to_string(), end.1.inverse() * frame)
        } else {
            (
                virtual_link(layer, &format!("carrier_{name}"), &pose),
                identity,
            )
        };
        let joint = format!(
            "{scope}/{}",
            unique_child(used_joints, &format!("base_{name}"))
        );
        let (type_name, drive, unit) = if angular {
            (
                "PhysicsRevoluteJoint",
                "angular",
                180.0 / std::f64::consts::PI,
            )
        } else {
            ("PhysicsPrismaticJoint", "linear", 1.0)
        };
        layer.ensure_prim(&joint, Specifier::Def, Some(type_name));
        layer.rel(&joint, "physics:body0", &parent);
        layer.rel(&joint, "physics:body1", &child);
        author_joint_frames(layer, &joint, &identity, &local1);
        token(layer, &joint, "physics:axis", axis);
        api_schemas(
            layer,
            &joint,
            &[
                &format!("PhysicsJointStateAPI:{drive}"),
                &format!("PhysicsDriveAPI:{drive}"),
            ],
        );
        // No limits and no force cap: the vehicle goes where it is told.
        float(
            layer,
            &joint,
            &format!("state:{drive}:physics:position"),
            value * unit,
        );
        float(
            layer,
            &joint,
            &format!("state:{drive}:physics:velocity"),
            0.0,
        );
        let ns = format!("drive:{drive}:physics");
        token(layer, &joint, &format!("{ns}:type"), "force");
        float(
            layer,
            &joint,
            &format!("{ns}:stiffness"),
            CARRIER_STIFFNESS / unit,
        );
        float(
            layer,
            &joint,
            &format!("{ns}:damping"),
            CARRIER_DAMPING / unit,
        );
        float(layer, &joint, &format!("{ns}:targetPosition"), value * unit);
        parent = child;
    }
}

pub(super) fn author_joint_frames(
    layer: &mut LayerBuilder,
    prim: &str,
    local0: &Isometry3<f64>,
    local1: &Isometry3<f64>,
) {
    for (k, pose) in [local0, local1].into_iter().enumerate() {
        layer.attr(
            prim,
            &format!("physics:localPos{k}"),
            "point3f",
            AttrValue::Default(vec3f(&pose.translation.vector)),
        );
        layer.attr(
            prim,
            &format!("physics:localRot{k}"),
            "quatf",
            AttrValue::Default(quatf(&pose.rotation)),
        );
    }
}
