//! Wheel appearance derived from the guided vehicle's checked motion.
//! No wheel forces, slip or collision geometry are introduced.
//!
//! Two kinds of wheel turn by the same closed form: a display obstacle of
//! a vehicle assembled from boxes ([`VehicleWheel`], decorated onto the
//! finished tracks), and a joint of a robot that *is* the vehicle's
//! running gear ([`crate::seq::WheelDrive`], turned tick by tick because
//! the joint track bakes alongside whatever else the robot does).

use botrail_model::{JointType, RobotModel};
use nalgebra::{Isometry3, Point3, Translation3, Unit, Vector3};
use serde::{Deserialize, Serialize};

use crate::rollout::{ObjectTrack, TrackSpan, VehiclePiece};
use crate::seq::{DeviceKind, WheelDrive};
use crate::Scene;

/// `|z|` of a wheel's axle direction: it must lie in the floor plane. The
/// catalog builder's `wheeled` check holds packages to the same number — a
/// vendor's `rpy="1.5708 0 0"` is already 4e-6 off level.
const AXLE_LEVEL: f64 = 0.02;
/// How far a steering axis may lean off the vertical (cos 2°).
const STEER_UP_MIN: f64 = 0.999_390_827_019_095_8;
/// Every wheel stands on one floor to within this, metres.
const FLOOR_TOL: f64 = 5e-3;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct VehicleWheel {
    pub object: String,
    /// Effective rolling radius in metres.
    pub radius: f64,
    /// Positive axle and its centre, both in the visual obstacle's local frame.
    pub axis: [f64; 3],
    pub pivot: [f64; 3],
    /// Lateral travel / rolling travel coupling: 0 for an ordinary wheel;
    /// -1 (front-left, rear-right) or +1 (front-right, rear-left) for
    /// the usual 45-degree mecanum arrangement, with axles pointing +Y.
    pub lateral_ratio: f64,
}

impl VehicleWheel {
    pub(crate) fn validate(&self, scene: &Scene, body: &[String]) -> Result<(), String> {
        let fail = |why: &str| format!("wheel `{}`: {why}", self.object);
        if !self.radius.is_finite() || self.radius <= 0.0 {
            return Err(fail("radius must be finite and positive"));
        }
        let axis_length = Vector3::from(self.axis).norm();
        if !self.axis.iter().chain(&self.pivot).all(|v| v.is_finite())
            || !self.lateral_ratio.is_finite()
            || !axis_length.is_finite()
            || axis_length < 1e-9
        {
            return Err(fail(
                "axis must be non-zero; axis, pivot and lateral_ratio must be finite",
            ));
        }
        if !body.contains(&self.object) {
            return Err(fail("visual must belong to the vehicle body"));
        }
        let Some(obstacle) = scene.obstacles().iter().find(|o| o.name == self.object) else {
            return Err(fail("unknown visual obstacle"));
        };
        if obstacle.enabled
            || obstacle
                .physics
                .as_ref()
                .is_some_and(|p| p.kind == botrail_physics::BodyKind::Dynamic)
        {
            return Err(fail(
                "use a disabled, non-dynamic visual; keep collisions separate",
            ));
        }
        let axle = obstacle.pose.rotation * Vector3::from(self.axis).normalize();
        if axle.z.abs() > 1e-6 {
            return Err(fail("the axle must be horizontal"));
        }
        Ok(())
    }
}

impl Scene {
    /// Register or replace one display wheel on an existing vehicle.
    pub fn set_vehicle_wheel(&mut self, vehicle: &str, wheel: VehicleWheel) -> Result<(), String> {
        let index = self
            .devices
            .iter()
            .position(|d| d.name == vehicle)
            .ok_or_else(|| format!("unknown vehicle `{vehicle}`"))?;
        let DeviceKind::Vehicle { body, .. } = &self.devices[index].kind else {
            return Err(format!("device `{vehicle}` is not a vehicle"));
        };
        wheel.validate(self, body)?;
        let DeviceKind::Vehicle { wheels, .. } = &mut self.devices[index].kind else {
            unreachable!()
        };
        if let Some(existing) = wheels.iter_mut().find(|w| w.object == wheel.object) {
            *existing = wheel;
        } else {
            wheels.push(wheel);
        }
        Ok(())
    }
}

/// Decorate the finished object tracks. The scan, sensors, planning and
/// collision checks all used the original rigid body poses.
pub(crate) fn animate(scene: &Scene, tracks: &mut [ObjectTrack]) {
    for device in scene.devices() {
        let DeviceKind::Vehicle { wheels, .. } = &device.kind else {
            continue;
        };
        for wheel in wheels {
            let Some(track) = tracks.iter_mut().find(|t| t.name == wheel.object) else {
                continue;
            };
            let axis = Unit::new_normalize(Vector3::from(wheel.axis));
            let pivot = Point3::from(wheel.pivot);
            let mut angle = 0.0;
            for span in &mut track.spans {
                let rate = angular_rate(span, wheel);
                let (t0, t1) = span.range();
                *span = TrackSpan::Wheel {
                    motion: Box::new(span.clone()),
                    axis,
                    pivot,
                    angle,
                    rate,
                };
                angle += rate * (t1 - t0);
            }
        }
    }
}

/// The centre's velocity projected onto forward + k * lateral, divided
/// by radius. A pivot's centre velocity includes its distance from the
/// turn centre; both axle and velocity rotate together, so the rate is
/// constant over each exact Linear/Pivot span.
fn angular_rate(span: &TrackSpan, wheel: &VehicleWheel) -> f64 {
    let (from, velocity) = match span {
        TrackSpan::Linear { from, velocity, .. } => (from, *velocity),
        TrackSpan::Pivot {
            from,
            center,
            omega,
            ..
        } => {
            let hub = from * Point3::from(wheel.pivot);
            (from, Vector3::z().cross(&(hub - center)) * *omega)
        }
        _ => return 0.0,
    };
    let axle = from.rotation * Vector3::from(wheel.axis).normalize();
    rolling_rate(&velocity, &axle, wheel.lateral_ratio, wheel.radius)
}

/// Signed rate about `axle` (world, unit) of a wheel whose hub travels at
/// `velocity`: positive turns the top of the wheel toward `axle x Z`, so
/// two wheels mirrored across the machine (axles +Y and -Y) both roll
/// forward, one counting up and one down. `lateral_ratio` does not change
/// sign with the axle — it is the rollers' handedness, not the joint's.
pub(crate) fn rolling_rate(
    velocity: &Vector3<f64>,
    axle: &Vector3<f64>,
    lateral_ratio: f64,
    radius: f64,
) -> f64 {
    let forward = axle.cross(&Vector3::z());
    velocity.dot(&(forward + axle * lateral_ratio)) / radius
}

/// One wheel of a [`WheelDrive`], resolved against its model.
#[derive(Debug, Clone)]
pub(crate) struct ResolvedWheel {
    /// The wheel joint: its model index and its slot in `q`.
    pub joint: usize,
    pub q: usize,
    pub radius: f64,
    pub lateral_ratio: f64,
    /// The steering joint, likewise.
    pub steer: Option<(usize, usize)>,
}

/// A [`WheelDrive`] resolved against its model.
#[derive(Debug, Clone)]
pub(crate) struct ResolvedWheelDrive {
    pub wheels: Vec<ResolvedWheel>,
    /// The mount offset that stands the machine on the vehicle plane with
    /// its turn centre over the vehicle's: the declared base frame on the
    /// vehicle frame, else the wheels' contact plane under their centroid,
    /// else nothing to go by (identity).
    pub offset: Isometry3<f64>,
}

impl ResolvedWheelDrive {
    /// Every `q` slot the mount drives: wheels and their steering.
    pub fn owned(&self) -> Vec<usize> {
        self.wheels
            .iter()
            .flat_map(|w| std::iter::once(w.q).chain(w.steer.map(|(_, q)| q)))
            .collect()
    }
}

/// Checks a [`WheelDrive`] against the model it is declared for — at
/// `current`, the configuration the machine is mounted in — and derives the
/// mount offset.
pub(crate) fn resolve_wheel_drive(
    model: &RobotModel,
    drive: &WheelDrive,
    current: &[f64],
) -> Result<ResolvedWheelDrive, String> {
    let poses = botrail_kin::forward_kinematics(model, current).map_err(|e| e.to_string())?;
    let actuated = |name: &str, role: &str| -> Result<(usize, usize), String> {
        let joint = model
            .joint_index(name)
            .ok_or_else(|| format!("{role} joint `{name}` is not a joint of this robot"))?;
        let q = model.joints[joint].q_index.ok_or_else(|| {
            format!(
                "{role} joint `{name}` carries no degree of freedom (fixed, or a mimic follower)"
            )
        })?;
        Ok((joint, q))
    };
    // The joint whose child is `link` — every link but the root has one.
    let parent_joint = |link: usize| model.joints.iter().position(|j| j.child_link == link);
    let mut wheels: Vec<ResolvedWheel> = Vec::with_capacity(drive.wheels.len());
    let mut taken: Vec<usize> = Vec::new();
    for wheel in &drive.wheels {
        let fail = |why: String| format!("wheel `{}`: {why}", wheel.joint);
        let (joint, q) = actuated(&wheel.joint, "wheel")?;
        let spec = &model.joints[joint];
        if matches!(spec.joint_type, JointType::Prismatic) || spec.limits.is_some() {
            return Err(fail(
                "a wheel joint must be continuous — position limits would be driven \
                 through their stops"
                    .into(),
            ));
        }
        if !(wheel.radius.is_finite() && wheel.radius > 0.0) {
            return Err(fail(format!(
                "radius must be finite and positive, got {}",
                wheel.radius
            )));
        }
        if !wheel.lateral_ratio.is_finite() {
            return Err(fail("lateral_ratio must be finite".into()));
        }
        let steer = wheel
            .steer
            .as_deref()
            .map(|name| actuated(name, "steer"))
            .transpose()
            .map_err(&fail)?;
        if let Some((steer_joint, _)) = steer {
            let steer_spec = &model.joints[steer_joint];
            if matches!(steer_spec.joint_type, JointType::Prismatic) {
                return Err(fail(format!(
                    "steer joint `{}` must turn, not slide",
                    steer_spec.name
                )));
            }
            let up = poses[steer_spec.child_link].rotation * steer_spec.axis.into_inner();
            if up.z.abs() < STEER_UP_MIN {
                return Err(fail(format!(
                    "steer joint `{}` must turn about the vertical, its axis points \
                     ({:.3}, {:.3}, {:.3})",
                    steer_spec.name, up.x, up.y, up.z
                )));
            }
            // The wheel hangs under its steering: walk up from the wheel.
            let mut link = spec.parent_link;
            let mut under = false;
            while let Some(j) = parent_joint(link) {
                if j == steer_joint {
                    under = true;
                    break;
                }
                link = model.joints[j].parent_link;
            }
            if !under {
                return Err(fail(format!(
                    "steer joint `{}` does not carry this wheel",
                    steer_spec.name
                )));
            }
        }
        let axle = poses[spec.child_link].rotation * spec.axis.into_inner();
        if axle.z.abs() > AXLE_LEVEL {
            return Err(fail(format!(
                "the axle must be level, it points ({:.3}, {:.3}, {:.3})",
                axle.x, axle.y, axle.z
            )));
        }
        for slot in std::iter::once(q).chain(steer.map(|(_, q)| q)) {
            if taken.contains(&slot) {
                return Err(fail("a joint may turn for one wheel only".into()));
            }
            taken.push(slot);
        }
        wheels.push(ResolvedWheel {
            joint,
            q,
            radius: wheel.radius,
            lateral_ratio: wheel.lateral_ratio,
            steer,
        });
    }
    let offset = match &drive.base_frame {
        Some(name) => {
            let frame = model
                .link_index(name)
                .ok_or_else(|| format!("base frame `{name}` is not a link of this robot"))?;
            // The frame that rides the vehicle must be rigid with the root
            // the vehicle moves: under a joint it would be left behind.
            let mut link = frame;
            while let Some(j) = parent_joint(link) {
                if !matches!(model.joints[j].joint_type, JointType::Fixed) {
                    return Err(format!(
                        "base frame `{name}` hangs under joint `{}`; it must be fixed to the \
                         root link `{}`",
                        model.joints[j].name, model.links[model.root_link].name
                    ));
                }
                link = model.joints[j].parent_link;
            }
            poses[frame].inverse()
        }
        None if wheels.is_empty() => Isometry3::identity(),
        None => {
            let hubs: Vec<(Point3<f64>, f64)> = wheels
                .iter()
                .map(|w| {
                    let hub = poses[model.joints[w.joint].child_link].translation.vector;
                    (Point3::from(hub), hub.z - w.radius)
                })
                .collect();
            let floor = hubs.iter().map(|(_, z)| *z).fold(f64::INFINITY, f64::min);
            if let Some((i, (_, z))) = hubs
                .iter()
                .enumerate()
                .find(|(_, (_, z))| z - floor > FLOOR_TOL)
            {
                return Err(format!(
                    "wheel `{}` stands {:.1} mm above the others' floor; state `base_frame`, \
                     or check the radii",
                    drive.wheels[i].joint,
                    (z - floor) * 1e3
                ));
            }
            let n = hubs.len() as f64;
            let (cx, cy) = hubs.iter().fold((0.0, 0.0), |(x, y), (hub, _)| {
                (x + hub.x / n, y + hub.y / n)
            });
            Translation3::new(-cx, -cy, -floor).into()
        }
    };
    Ok(ResolvedWheelDrive { wheels, offset })
}

/// A gait and a wheel drive checked as one machine — a wheel-legged
/// quadruped (design-wheel-legged.md §3.2): every wheel hangs from a foot,
/// the foot frame sits on the axle, and the wheel's radius is the height
/// the gait stands that foot at. The legs stay the gait's and the wheel
/// joints the drive's; neither knows the other exists.
#[derive(Debug, Clone)]
pub(crate) struct ResolvedWheelLegs {
    pub gait: crate::gait::ResolvedGait,
    /// The mount offset for the declared mode: the vehicle plane under
    /// the root, the hubs one radius above it — read off the stance for a
    /// walker, off the posture the machine stands in now for a roller.
    pub offset: Isometry3<f64>,
}

/// Millimetre: how far a wheel's hub may sit from its foot frame, and how
/// far the radius may differ from the gait's foot height.
const HUB_ON_FOOT: f64 = 1e-3;

/// Checks a gait and a wheel drive declared for one model as the two
/// halves of a wheel-legged machine, at `current` — the configuration it is
/// mounted in, its travel posture when it rolls.
pub(crate) fn resolve_wheel_legs(
    model: &RobotModel,
    gait: &crate::seq::GaitSpec,
    drive: &WheelDrive,
    current: &[f64],
) -> Result<ResolvedWheelLegs, String> {
    let gait = crate::gait::resolve_gait(model, gait, current).map_err(|m| format!("gait: {m}"))?;
    if drive.wheels.len() != gait.legs.len() {
        return Err(format!(
            "wheels: {} wheels for {} legs — a wheel-legged machine has one wheel on every foot",
            drive.wheels.len(),
            gait.legs.len()
        ));
    }
    for wheel in &drive.wheels {
        if wheel.steer.is_some() {
            return Err(format!(
                "wheels: wheel `{}` names a steer joint — a wheel-legged machine steers with \
                 its legs; `steer` is a swerve module's",
                wheel.joint
            ));
        }
        if wheel.lateral_ratio != 0.0 {
            return Err(format!(
                "wheels: wheel `{}` has lateral_ratio {} — a leg's wheel has no rollers",
                wheel.joint, wheel.lateral_ratio
            ));
        }
    }
    // The drive against the model at the stance — and, for a roller, at
    // the posture it stands in now, which is how it will travel: continuous
    // joints, level axles, all hubs on one floor. The floor is checked from
    // the hubs, whatever `base_frame` says — the frame is validated on its
    // own, and the postures decide the offset here.
    let by_hubs = WheelDrive {
        base_frame: None,
        ..drive.clone()
    };
    let checked = resolve_wheel_drive(model, &by_hubs, &gait.stance)
        .map_err(|m| format!("wheels (at the stance): {m}"))?;
    if drive.mode == crate::seq::LocomotionMode::Roll {
        resolve_wheel_drive(model, &by_hubs, current)
            .map_err(|m| format!("wheels (as mounted, to roll): {m}"))?;
    }
    if drive.base_frame.is_some() {
        resolve_wheel_drive(model, drive, &gait.stance).map_err(|m| format!("wheels: {m}"))?;
    }
    let rigid_root = |link: usize| rigid_root(model, link);
    let poses = botrail_kin::forward_kinematics(model, &gait.stance).map_err(|e| e.to_string())?;
    let mut carried: Vec<Option<usize>> = vec![None; gait.legs.len()];
    for (w, wheel) in checked.wheels.iter().enumerate() {
        let spec = &model.joints[wheel.joint];
        let name = &spec.name;
        // A foot that *is* the wheel puts the wheel joint in the leg's
        // chain: the gait would own it and the stance would have to name
        // it. The foot is the axle the wheel turns on.
        if let Some(leg) = gait.legs.iter().find(|l| l.joints.contains(&wheel.q)) {
            return Err(format!(
                "wheels: leg `{}`'s foot `{}` hangs under wheel joint `{name}` — the foot must \
                 be the axle frame the wheel turns on (a frame fixed to `{}`), not the wheel",
                leg.name, model.links[leg.foot].name, model.links[spec.parent_link].name
            ));
        }
        let hub_root = rigid_root(spec.parent_link);
        let Some(i) = gait
            .legs
            .iter()
            .position(|l| rigid_root(l.foot) == hub_root)
        else {
            return Err(format!(
                "wheels: wheel `{name}` turns on `{}`, which is rigid with no foot — every \
                 wheel of a wheel-legged machine hangs from a leg's foot link",
                model.links[spec.parent_link].name
            ));
        };
        if let Some(other) = carried[i] {
            return Err(format!(
                "wheels: leg `{}` carries two wheels, `{}` and `{name}`",
                gait.legs[i].name, model.joints[checked.wheels[other].joint].name
            ));
        }
        carried[i] = Some(w);
        let hub = poses[spec.child_link].translation.vector;
        let foot = gait.legs[i].nominal.translation.vector;
        let apart = (hub - foot).norm();
        if apart > HUB_ON_FOOT {
            return Err(format!(
                "wheels: wheel `{name}` turns {:.1} mm from foot `{}`'s origin — the foot \
                 frame must sit on the axle",
                apart * 1e3,
                model.links[gait.legs[i].foot].name
            ));
        }
        if (wheel.radius - gait.foot_radius).abs() > HUB_ON_FOOT {
            return Err(format!(
                "wheels: wheel `{name}` has radius {} m and the gait stands its feet {} m \
                 above the floor (`foot_radius`) — on a wheel-legged machine these are one \
                 number, the wheel's",
                wheel.radius, gait.foot_radius
            ));
        }
    }
    // Set to roll, the machine is mounted as it stands (its travel
    // posture): the vehicle plane is a wheel radius under the hubs there.
    // Walking, or deciding leg by leg, it is mounted in its stance — the
    // posture it rolls in too, between walks — where the gait already read
    // the plane.
    let offset = match drive.mode {
        crate::seq::LocomotionMode::Auto | crate::seq::LocomotionMode::Walk => gait.offset,
        crate::seq::LocomotionMode::Roll => {
            let now = botrail_kin::forward_kinematics(model, current).map_err(|e| e.to_string())?;
            let mean = gait
                .legs
                .iter()
                .map(|l| now[l.foot].translation.z)
                .sum::<f64>()
                / gait.legs.len() as f64;
            Translation3::new(0.0, 0.0, gait.foot_radius - mean).into()
        }
    };
    Ok(ResolvedWheelLegs { gait, offset })
}

/// The rigid group a link belongs to: up through fixed joints to the first
/// link under a moving joint (or the root).
fn rigid_root(model: &RobotModel, mut link: usize) -> usize {
    while let Some(ji) = model.links[link].parent_joint {
        if !matches!(model.joints[ji].joint_type, JointType::Fixed) {
            break;
        }
        link = model.joints[ji].parent_link;
    }
    link
}

/// The wheel of `drive` that hangs from the foot link `foot` — the one
/// whose joint turns on a link rigid with it — as an index into
/// `drive.wheels`.
pub(crate) fn wheel_of_foot(
    model: &RobotModel,
    drive: &ResolvedWheelDrive,
    foot: usize,
) -> Option<usize> {
    let root = rigid_root(model, foot);
    drive
        .wheels
        .iter()
        .position(|w| rigid_root(model, model.joints[w.joint].parent_link) == root)
}

/// How far each wheel (and where each steering joint) turns while its
/// machine's base, at `from`, rides `piece` for `dt`: `(q slot, new value)`
/// pairs, applied onto `q`. The hub velocity and the axle turn together
/// through a pivot, so the rate is exact over the whole piece.
pub(crate) fn roll(
    model: &RobotModel,
    drive: &ResolvedWheelDrive,
    q: &mut [f64],
    from: &Isometry3<f64>,
    piece: &VehiclePiece,
    dt: f64,
) {
    let Ok(local) = botrail_kin::forward_kinematics(model, q) else {
        return;
    };
    for wheel in &drive.wheels {
        let spec = &model.joints[wheel.joint];
        let hub = from * Point3::from(local[spec.child_link].translation.vector);
        let velocity = match piece {
            VehiclePiece::Lin { velocity } => *velocity,
            VehiclePiece::Piv { center, omega } => Vector3::z().cross(&(hub - center)) * *omega,
        };
        let mut axle = from.rotation * (local[spec.child_link].rotation * spec.axis.into_inner());
        if let Some((steer_joint, steer_q)) = wheel.steer {
            let travel = Vector3::new(velocity.x, velocity.y, 0.0);
            if travel.norm() > 1e-9 {
                // Aim the rolling direction along the hub's travel — or
                // against it, whichever is the shorter turn: a wheel rolls
                // backward as well as forward.
                let steer_spec = &model.joints[steer_joint];
                let up = from.rotation
                    * (local[steer_spec.child_link].rotation * steer_spec.axis.into_inner());
                let forward = axle.cross(&Vector3::z());
                let turn = forward.y.atan2(forward.x);
                let mut delta = travel.y.atan2(travel.x) - turn;
                delta = (delta + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                if delta > std::f64::consts::FRAC_PI_2 {
                    delta -= std::f64::consts::PI;
                } else if delta < -std::f64::consts::FRAC_PI_2 {
                    delta += std::f64::consts::PI;
                }
                // About a steering axis that points down, the same turn
                // in the world is the opposite joint travel.
                let signed = delta * up.z.signum();
                let aimed = match steer_spec.limits {
                    Some(l) => (q[steer_q] + signed).clamp(l.lower, l.upper),
                    None => q[steer_q] + signed,
                };
                let applied = (aimed - q[steer_q]) * up.z.signum();
                q[steer_q] = aimed;
                axle =
                    nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), applied) * axle;
            }
        }
        q[wheel.q] += rolling_rate(&velocity, &axle, wheel.lateral_ratio, wheel.radius) * dt;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rollout::SequenceTimeline;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};

    #[test]
    fn mecanum_rates_include_sideways_travel_and_turn_radius() {
        for (x, y, lateral) in [
            (0.2, 0.3, -1.0),
            (0.2, -0.3, 1.0),
            (-0.2, 0.3, 1.0),
            (-0.2, -0.3, -1.0),
        ] {
            let wheel = VehicleWheel {
                object: "wheel".into(),
                radius: 0.1,
                axis: [0.0, 1.0, 0.0],
                pivot: [0.0; 3],
                lateral_ratio: lateral,
            };
            let from = Isometry3::translation(x, y, 0.1);
            let straight = TrackSpan::Linear {
                t0: 0.0,
                t1: 1.0,
                from,
                velocity: Vector3::new(-0.5, 0.2, 0.0),
            };
            assert!((angular_rate(&straight, &wheel) - (-5.0 + lateral * 2.0)).abs() < 1e-9);
            let turn = TrackSpan::Pivot {
                t0: 0.0,
                t1: 1.0,
                from,
                center: Point3::origin(),
                omega: 1.0,
            };
            // A positive yaw turns both left wheels backwards and both
            // right wheels forwards, at (half_length + half_width) / r.
            let expected = if y > 0.0 { -5.0 } else { 5.0 };
            assert!((angular_rate(&turn, &wheel) - expected).abs() < 1e-9);
        }
    }

    #[test]
    fn local_pivot_and_axle_work_in_a_rotated_visual_frame() {
        let from = Isometry3::from_parts(
            Translation3::new(0.2, 0.3, 0.1),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
        );
        let pivot = Point3::new(0.02, 0.0, 0.0);
        let motion = TrackSpan::Linear {
            t0: 2.0,
            t1: 3.0,
            from,
            velocity: Vector3::new(0.4, 0.0, 0.0),
        };
        let wheel = VehicleWheel {
            object: "wheel".into(),
            radius: 0.1,
            axis: [1.0, 0.0, 0.0],
            pivot: pivot.into(),
            lateral_ratio: 0.0,
        };
        let rate = angular_rate(&motion, &wheel);
        assert!((rate - 4.0).abs() < 1e-9);
        let span = TrackSpan::Wheel {
            motion: Box::new(motion),
            axis: Vector3::x_axis(),
            pivot,
            angle: 1.3,
            rate,
        };
        for t in [2.8, 2.0, 2.4, 3.0, 3.5] {
            let pose = SequenceTimeline::span_pose(std::slice::from_ref(&span), &[], t).unwrap();
            let elapsed = (t - 2.0).clamp(0.0, 1.0);
            let expected_hub = from * pivot + Vector3::new(0.4 * elapsed, 0.0, 0.0);
            assert!((pose * pivot - expected_hub).norm() < 1e-9);
            let relative = from.rotation.inverse() * pose.rotation;
            let expected =
                UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 1.3 + rate * elapsed);
            assert!(relative.angle_to(&expected) < 1e-9);
        }
    }
}
