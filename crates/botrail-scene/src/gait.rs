//! Kinematic gaits: a robot mounted on a vehicle with a gait walks whenever
//! that vehicle drives. There is no physics in it — the body rides the
//! vehicle's closed-form motion exactly as an AMR's base does, the footfalls
//! are planned from that motion the moment the vehicle is dispatched, a
//! planted foot never moves in the world, and the legs are solved by per-leg
//! IK every scan tick. What it answers is what the vehicle answered already
//! — does it fit, does it clash, how long does it take — with legs that move
//! like legs instead of a body that hovers. See design/design-legged.md.

use std::f64::consts::PI;

use botrail_kin::{IkMode, IkOptions};
use botrail_model::RobotModel;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use crate::rollout::{apply_piece, VehiclePiece};
use crate::seq::{FootContact, GaitPattern, GaitSpec};

/// One landing of a foot on a baked timeline: the foot left its previous
/// anchor at `lift`, flew, and has stood at `position` since `land`.
#[derive(Debug, Clone, PartialEq)]
pub struct Footfall {
    /// Leg name, as the gait declared it (`FL`, `L`, ...).
    pub leg: String,
    pub lift: f64,
    pub land: f64,
    /// World position of the foot link's origin while planted.
    pub position: Point3<f64>,
    /// The body's heading as the foot landed. A sole points this way (plus
    /// its stance offset) for as long as it stands — a pivot turns the body
    /// over planted feet, and the feet follow it one landing at a time.
    pub yaw: f64,
}

/// The body's bob and lean over one walk, as a closed-form offset composed
/// onto the base track: the body rises over each planted leg and leans
/// toward it, the legs absorb the difference, and the feet stay put. Zero
/// amplitudes are no sway at all.
#[derive(Debug, Clone, PartialEq)]
pub struct BodySway {
    pub t0: f64,
    /// When the walk ends (settle included): the sway has faded out by here.
    pub done: f64,
    pub period: f64,
    /// Swing duration — the bob peaks at the first leg's mid-swing.
    pub swing: f64,
    /// Vertical amplitude (m), twice per cycle.
    pub bob: f64,
    /// Lateral amplitude (m), once per cycle.
    pub lateral: f64,
    /// Which way (`±1` along the body's y) the body leans while the first
    /// leg swings: away from it, onto the planted one.
    pub lean: f64,
}

fn smooth(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

impl BodySway {
    /// The offset at `t`, in the body frame (`base ∘ offset`). Fades in
    /// over the first half period and out over the last, so the body
    /// leaves its rigid ride and returns to it without a step.
    pub fn offset_at(&self, t: f64) -> Isometry3<f64> {
        if t <= self.t0 || t >= self.done {
            return Isometry3::identity();
        }
        let half = 0.5 * self.period;
        let envelope = smooth((t - self.t0) / half) * smooth((self.done - t) / half);
        let phase = 2.0 * PI * (t - self.t0) / self.period - PI * self.swing / self.period;
        let z = self.bob * (2.0 * phase).cos();
        let y = self.lean * self.lateral * phase.cos();
        Isometry3::from_parts(
            Translation3::new(0.0, y * envelope, z * envelope),
            UnitQuaternion::identity(),
        )
    }
}

/// The sway offset of the walk covering `t`, identity between walks.
pub fn sway_offset(sways: &[BodySway], t: f64) -> Isometry3<f64> {
    sways
        .iter()
        .find(|s| t > s.t0 && t < s.done)
        .map(|s| s.offset_at(t))
        .unwrap_or_else(Isometry3::identity)
}

/// A leg of a gait, resolved against the model it walks.
#[derive(Debug, Clone)]
pub(crate) struct ResolvedLeg {
    pub name: String,
    /// Foot link index.
    pub foot: usize,
    /// The leg's own DOF (q indices): what the per-leg solve may move.
    pub joints: Vec<usize>,
    /// Foot pose in the root frame at the stance — where the foot rests
    /// under the body when the machine stands.
    pub nominal: Isometry3<f64>,
    pub ik: IkOptions,
    /// For a yaw-free sole: the hip yaw, seeded analytically each tick.
    pub yaw_seed: Option<YawSeed>,
}

/// A 5-DOF leg keeps its sole level only when the leg's plane contains the
/// foot — the hip yaw has to point the plane at it, and the direction it
/// must point swings through a half turn as the foot passes under the hip.
/// A local solve creeps there; this seeds the yaw with the answer.
#[derive(Debug, Clone)]
pub(crate) struct YawSeed {
    /// The hip yaw joint's q index, parent link, and origin (its frame at
    /// zero, in which the axis is fixed).
    pub joint: usize,
    pub parent: usize,
    pub origin: Isometry3<f64>,
    /// `+1` when the joint's axis is its frame's +Z, `-1` for -Z.
    pub sign: f64,
    /// The leg plane's direction in the joint frame at the stance (the
    /// pitch joints move the foot along it), and the yaw that set it.
    pub u0: nalgebra::Vector2<f64>,
    pub y0: f64,
}

impl YawSeed {
    /// The yaw that points the leg's plane at a foot `target` (world),
    /// given the parent link's world pose. The plane may contain the foot
    /// ahead or behind; whichever keeps the yaw nearer the stance's wins,
    /// so the plane never flips over and the knee keeps bending its way.
    /// `None` with the foot (nearly) under the hip, where any yaw does.
    pub fn yaw_for(&self, parent_pose: &Isometry3<f64>, target: &Point3<f64>) -> Option<f64> {
        let frame = parent_pose * self.origin;
        let local = frame.inverse().transform_point(target);
        let u = nalgebra::Vector2::new(local.x, local.y);
        if u.norm() < 5e-3 {
            return None;
        }
        let turn = |v: nalgebra::Vector2<f64>| {
            let angle = (self.u0.x * v.y - self.u0.y * v.x).atan2(self.u0.dot(&v));
            self.sign * angle
        };
        let (a, b) = (turn(u), turn(-u));
        Some(self.y0 + if a.abs() <= b.abs() { a } else { b })
    }
}

/// A [`GaitSpec`] checked against a model: every name resolved, the stance
/// complete, the pattern's phase table laid out over the declared legs.
#[derive(Debug, Clone)]
pub(crate) struct ResolvedGait {
    pub legs: Vec<ResolvedLeg>,
    /// The standing configuration (full q).
    pub stance: Vec<f64>,
    pub period: f64,
    pub duty: f64,
    /// Phase of each leg's cycle, indexed like `legs`.
    pub phases: Vec<f64>,
    pub lift: f64,
    pub max_stride: f64,
    /// `(q index, amplitude)` of the joints swung in time with leg 0.
    pub arm_swing: Vec<(usize, f64)>,
    /// Body sway amplitudes (m): vertical, lateral.
    pub bob: f64,
    pub lateral: f64,
    /// Lean direction while leg 0 swings (`±1` along body y).
    pub lean: f64,
    /// Mount offset that stands the stance feet on the vehicle plane: the
    /// root lifted by the feet's depth below it (plus the foot radius).
    pub offset: Isometry3<f64>,
}

impl ResolvedGait {
    /// Every leg DOF.
    pub fn leg_joints(&self) -> Vec<usize> {
        self.legs.iter().flat_map(|l| l.joints.clone()).collect()
    }

    /// Swing duration: the part of a cycle a foot is in the air.
    pub fn swing(&self) -> f64 {
        (1.0 - self.duty) * self.period
    }

    /// A foot's world pose for a planted position and body heading.
    pub fn foot_pose(&self, leg: usize, position: &Point3<f64>, yaw: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::from(position.coords),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw)
                * self.legs[leg].nominal.rotation,
        )
    }
}

/// Per-leg solve: position only for a point foot, the sole flat on the
/// floor for a sole (yaw pinned or free by the leg's DOF). Warm-started
/// every tick, so the tolerance can be tight — it *is* the foot's slip
/// (half a micrometre; a foot straight under its hip is a singular pose
/// for a yaw-free leg, where the damped solve creeps its last decade).
/// No restarts and no centering, for the same reasons as the conveyor
/// track solve: a different branch or a self-motion drift would show up
/// as a leg that flips or wanders mid-stride.
fn leg_ik(mode: IkMode) -> IkOptions {
    IkOptions {
        mode,
        max_iters: 100,
        tol_pos: 5e-7,
        tol_rot: 1e-5,
        // A yaw-free sole's two tasks (place the foot, keep it level) pull
        // against each other through the hip roll whenever the foot moves
        // sideways in the leg's plane — a pivot — and a lightly damped step
        // overshoots between them. More damping, and it settles.
        damping: if mode == IkMode::Axis { 0.08 } else { 0.01 },
        orientation_weight: 0.5,
        max_step: 0.3,
        restarts: 0,
        null_space_gain: 0.0,
    }
}

/// Legs a foot rests on, in declaration order, for the built-in patterns.
fn phase_table(pattern: &GaitPattern, legs: usize) -> Result<(f64, Vec<f64>), String> {
    match pattern {
        // Lateral sequence: FL, RR, FR, RL — declared FL, FR, RL, RR.
        GaitPattern::Walk => {
            if legs != 4 {
                return Err(format!("the walk pattern is for 4 legs, got {legs}"));
            }
            Ok((0.75, vec![0.0, 0.5, 0.75, 0.25]))
        }
        // Diagonal pairs in antiphase.
        GaitPattern::Trot => {
            if legs != 4 {
                return Err(format!("the trot pattern is for 4 legs, got {legs}"));
            }
            Ok((0.5, vec![0.0, 0.5, 0.5, 0.0]))
        }
        GaitPattern::Biped => {
            if legs != 2 {
                return Err(format!("the biped pattern is for 2 legs, got {legs}"));
            }
            Ok((0.6, vec![0.0, 0.5]))
        }
        GaitPattern::Custom { duty, phases } => {
            if !(duty.is_finite() && *duty > 0.0 && *duty < 1.0) {
                return Err(format!("duty must be in (0, 1), got {duty}"));
            }
            if phases.len() != legs {
                return Err(format!(
                    "custom pattern has {} phases for {legs} legs",
                    phases.len()
                ));
            }
            for p in phases {
                if !(p.is_finite() && *p >= 0.0 && *p < 1.0) {
                    return Err(format!("phases must be in [0, 1), got {p}"));
                }
            }
            Ok((*duty, phases.clone()))
        }
    }
}

/// Checks `spec` against `model` and lays it out for the scan engine.
/// `current` is the robot's configuration at mount time: joints the stance
/// does not name keep it.
pub(crate) fn resolve_gait(
    model: &RobotModel,
    spec: &GaitSpec,
    current: &[f64],
) -> Result<ResolvedGait, String> {
    if current.len() != model.dof() {
        return Err(format!(
            "expected {} joint positions, got {}",
            model.dof(),
            current.len()
        ));
    }
    if spec.legs.len() < 2 {
        return Err(format!(
            "a gait needs at least 2 legs, got {}",
            spec.legs.len()
        ));
    }
    for (name, limit) in [
        ("period", spec.period),
        ("lift", spec.lift),
        ("max_stride", spec.max_stride),
    ] {
        if !(limit.is_finite() && limit > 0.0) {
            return Err(format!("gait {name} must be positive, got {limit}"));
        }
    }
    for (name, value) in [
        ("foot_radius", spec.foot_radius),
        ("bob", spec.bob),
        ("lateral", spec.lateral),
    ] {
        if !(value.is_finite() && value >= 0.0) {
            return Err(format!("gait {name} must be non-negative, got {value}"));
        }
    }
    let body = match &spec.body_link {
        Some(name) => model
            .link_index(name)
            .ok_or_else(|| format!("unknown body link `{name}`"))?,
        None => model.root_link,
    };

    // The stance: named joints over the mount-time configuration.
    let mut stance = current.to_vec();
    let mut named: Vec<usize> = Vec::new();
    for (joint, value) in &spec.stance {
        let ji = model
            .joint_index(joint)
            .ok_or_else(|| format!("stance names unknown joint `{joint}`"))?;
        let qi = model.joints[ji]
            .q_index
            .ok_or_else(|| format!("stance joint `{joint}` is not actuated"))?;
        if !value.is_finite() {
            return Err(format!("stance value for `{joint}` is not finite"));
        }
        if let Some(limits) = model.joints[ji].limits {
            if *value < limits.lower - 1e-9 || *value > limits.upper + 1e-9 {
                return Err(format!(
                    "stance value {value} for `{joint}` is outside its limits \
                     [{}, {}]",
                    limits.lower, limits.upper
                ));
            }
        }
        stance[qi] = *value;
        named.push(qi);
    }

    let trunk = model.driving_joints(body);
    let mut legs = Vec::with_capacity(spec.legs.len());
    let mut taken: Vec<usize> = Vec::new();
    for leg in &spec.legs {
        if legs.iter().any(|l: &ResolvedLeg| l.name == leg.name) {
            return Err(format!("leg `{}` is declared twice", leg.name));
        }
        let foot = model
            .link_index(&leg.foot)
            .ok_or_else(|| format!("leg `{}`: unknown foot link `{}`", leg.name, leg.foot))?;
        // The foot must hang below the body link.
        let mut cursor = foot;
        let mut under_body = cursor == body;
        while let Some(ji) = model.links[cursor].parent_joint {
            cursor = model.joints[ji].parent_link;
            if cursor == body {
                under_body = true;
                break;
            }
        }
        if !under_body {
            return Err(format!(
                "leg `{}`: foot `{}` does not hang from body link `{}`",
                leg.name, leg.foot, model.links[body].name
            ));
        }
        let joints: Vec<usize> = model
            .driving_joints(foot)
            .into_iter()
            .filter(|ji| !trunk.contains(ji))
            .map(|ji| {
                model.joints[ji]
                    .q_index
                    .expect("driving joints are actuated")
            })
            .collect();
        if joints.is_empty() {
            return Err(format!(
                "leg `{}`: no actuated joint between `{}` and `{}`",
                leg.name, model.links[body].name, leg.foot
            ));
        }
        if let Some(qi) = joints.iter().find(|qi| taken.contains(qi)) {
            let name = &model.joints[model.actuated_joints[*qi]].name;
            return Err(format!(
                "leg `{}` shares joint `{name}` with another leg",
                leg.name
            ));
        }
        if let Some(qi) = joints.iter().find(|qi| !named.contains(qi)) {
            let name = &model.joints[model.actuated_joints[*qi]].name;
            return Err(format!(
                "stance does not name leg `{}`'s joint `{name}` — the stance must \
                 place every leg joint",
                leg.name
            ));
        }
        let mode = match leg.contact {
            FootContact::Point => IkMode::Position,
            FootContact::Sole { .. } => match joints.len() {
                5 => IkMode::Axis,
                n if n >= 6 => IkMode::Pose,
                n => {
                    return Err(format!(
                        "leg `{}`: a sole needs 5 or 6 DOF to stay flat, this leg has {n}",
                        leg.name
                    ))
                }
            },
        };
        taken.extend(joints.iter().copied());
        legs.push(ResolvedLeg {
            name: leg.name.clone(),
            foot,
            joints,
            nominal: Isometry3::identity(),
            ik: leg_ik(mode),
            yaw_seed: None,
        });
    }

    let (duty, phases) = phase_table(&spec.pattern, legs.len())?;

    let mut arm_swing = Vec::with_capacity(spec.arm_swing.len());
    for (joint, amplitude) in &spec.arm_swing {
        let ji = model
            .joint_index(joint)
            .ok_or_else(|| format!("arm_swing names unknown joint `{joint}`"))?;
        let qi = model.joints[ji]
            .q_index
            .ok_or_else(|| format!("arm_swing joint `{joint}` is not actuated"))?;
        if taken.contains(&qi) {
            return Err(format!("arm_swing joint `{joint}` belongs to a leg"));
        }
        if !amplitude.is_finite() {
            return Err(format!("arm_swing amplitude for `{joint}` is not finite"));
        }
        arm_swing.push((qi, *amplitude));
    }

    // Where the feet rest at the stance, and how far below the root: the
    // default mount lifts the root so they sit on the vehicle plane.
    let poses = botrail_kin::forward_kinematics(model, &stance).map_err(|e| e.to_string())?;
    for (leg, spec_leg) in legs.iter_mut().zip(&spec.legs) {
        leg.nominal = poses[leg.foot];
        if leg.ik.mode == IkMode::Axis {
            leg.yaw_seed = yaw_seed(model, leg, &stance, &poses);
        }
        // A sole is driven flat by its +Z: the solve can only keep it level
        // if that is the axis that points up when the machine stands.
        if matches!(spec_leg.contact, FootContact::Sole { .. }) {
            let up = (leg.nominal.rotation * Vector3::z()).z;
            if up < (5.0f64).to_radians().cos() {
                return Err(format!(
                    "leg `{}`: a sole's foot link must point +Z up in the stance, `{}` is \
                     tilted {:.1}° — give the model a level sole frame",
                    leg.name,
                    spec_leg.foot,
                    up.clamp(-1.0, 1.0).acos().to_degrees()
                ));
            }
        }
    }
    let zs: Vec<f64> = legs.iter().map(|l| l.nominal.translation.z).collect();
    let mean = zs.iter().sum::<f64>() / zs.len() as f64;
    if let Some((leg, z)) = legs.iter().zip(&zs).find(|(_, z)| (*z - mean).abs() > 5e-3) {
        return Err(format!(
            "the stance does not stand level: foot `{}` sits at z = {:.4} against a mean \
             of {mean:.4} (feet must lie on one plane within 5 mm)",
            leg.name, z
        ));
    }
    let offset = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, spec.foot_radius - mean),
        UnitQuaternion::identity(),
    );
    // Lean away from the first leg while it swings — onto whichever side
    // the other feet are.
    let lean = if legs[0].nominal.translation.y > 0.0 {
        -1.0
    } else {
        1.0
    };

    Ok(ResolvedGait {
        legs,
        stance,
        period: spec.period,
        duty,
        phases,
        lift: spec.lift,
        max_stride: spec.max_stride,
        arm_swing,
        bob: spec.bob,
        lateral: spec.lateral,
        lean,
        offset,
    })
}

/// The hip yaw of a yaw-free sole's leg, for seeding: the leg joint nearest
/// the body whose axis stands vertical at the stance, with the leg plane
/// read off the pitch joints (the axis most of the leg's joints share,
/// horizontal at the stance). `None` when the leg is not built that way —
/// the solve then runs unseeded.
fn yaw_seed(
    model: &RobotModel,
    leg: &ResolvedLeg,
    stance: &[f64],
    poses: &[Isometry3<f64>],
) -> Option<YawSeed> {
    let vertical = (5.0f64).to_radians().cos();
    let depth = |ji: usize| {
        let mut depth = 0;
        let mut cursor = model.joints[ji].parent_link;
        while let Some(pj) = model.links[cursor].parent_joint {
            depth += 1;
            cursor = model.joints[pj].parent_link;
        }
        depth
    };
    // Axes in the root frame at the stance: a joint's axis lives in its
    // child link's frame.
    let axis_at = |ji: usize| {
        poses[model.joints[ji].child_link].rotation * model.joints[ji].axis.into_inner()
    };
    let joints: Vec<usize> = leg
        .joints
        .iter()
        .map(|&qi| model.actuated_joints[qi])
        .collect();
    let yaw = *joints
        .iter()
        .filter(|&&ji| axis_at(ji).z.abs() > vertical)
        .min_by_key(|&&ji| depth(ji))?;
    // The pitch family: the horizontal axis most joints share.
    let mut best: Option<(usize, Vector3<f64>)> = None;
    for &ji in &joints {
        let a = axis_at(ji);
        if a.z.abs() > (1.0 - vertical).sqrt() {
            continue;
        }
        let count = joints
            .iter()
            .filter(|&&jk| axis_at(jk).dot(&a).abs() > vertical)
            .count();
        if best.as_ref().is_none_or(|(n, _)| count > *n) {
            best = Some((count, a));
        }
    }
    let (count, normal) = best?;
    if count < 2 {
        return None;
    }
    let joint = &model.joints[yaw];
    let frame = poses[joint.parent_link] * joint.origin;
    let normal_local = frame.rotation.inverse() * normal;
    let along = Vector3::z().cross(&normal_local);
    let u0 = nalgebra::Vector2::new(along.x, along.y);
    if u0.norm() < 1e-6 {
        return None;
    }
    Some(YawSeed {
        joint: joint.q_index?,
        parent: joint.parent_link,
        origin: joint.origin,
        sign: joint.axis.z.signum(),
        u0: u0.normalize(),
        y0: stance[joint.q_index?],
    })
}

/// The stride the vehicle's rates ask of the legs, against what the gait
/// allows: a straight leg moves a foot `speed · period` between landings,
/// a pivot swings the outermost foot `turn_speed · period · r` around the
/// vehicle origin. Either beyond `max_stride` is an authoring error — the
/// cure is a slower vehicle or a shorter period, named here rather than
/// discovered as an IK failure mid-walk.
pub(crate) fn check_stride(
    gait: &ResolvedGait,
    offset: &Isometry3<f64>,
    speed: f64,
    turn_speed: f64,
) -> Result<(), String> {
    let stride = speed * gait.period;
    if stride > gait.max_stride + 1e-9 {
        return Err(format!(
            "a vehicle speed of {speed} m/s with a gait period of {} s asks for a stride \
             of {stride:.3} m, beyond the gait's max_stride {:.3} m — lower the speed or \
             shorten the period",
            gait.period, gait.max_stride
        ));
    }
    let radius = gait
        .legs
        .iter()
        .map(|l| {
            let p = offset * l.nominal.translation.vector;
            (p.x * p.x + p.y * p.y).sqrt()
        })
        .fold(0.0, f64::max);
    let arc = turn_speed * gait.period * radius;
    if arc > gait.max_stride + 1e-9 {
        return Err(format!(
            "a turn rate of {turn_speed} rad/s swings the outer feet {arc:.3} m per gait \
             period, beyond the gait's max_stride {:.3} m — lower turn_speed or shorten \
             the period",
            gait.max_stride
        ));
    }
    Ok(())
}

/// The vehicle's motion over one dispatch, closed form: the legs of the
/// route with the frame each starts from. What the footfalls are planned
/// against — the same pieces the body is driven by, so the plan and the
/// drive cannot disagree.
#[derive(Debug, Clone)]
pub(crate) struct BodyProfile {
    pub t0: f64,
    pub t_end: f64,
    /// `(start, end, vehicle frame at start, motion)`, tiling `[t0, t_end]`.
    pub pieces: Vec<(f64, f64, Isometry3<f64>, VehiclePiece)>,
    pub end_frame: Isometry3<f64>,
}

impl BodyProfile {
    /// The vehicle frame at `t` — held at the ends beyond the drive.
    pub fn frame_at(&self, t: f64) -> Isometry3<f64> {
        if t >= self.t_end {
            return self.end_frame;
        }
        for (a, b, frame, piece) in &self.pieces {
            if t < *b {
                return apply_piece(frame, piece, (t - a).max(0.0));
            }
        }
        self.end_frame
    }
}

/// Heading of a frame about +Z.
pub(crate) fn yaw_of(frame: &Isometry3<f64>) -> f64 {
    let r = frame.rotation.to_rotation_matrix();
    r[(1, 0)].atan2(r[(0, 0)])
}

/// One leg's planned footfalls over a dispatch.
#[derive(Debug, Clone)]
pub(crate) struct LegPlan {
    /// Where the foot stood at dispatch, and the body heading it stood at.
    pub start: Point3<f64>,
    pub start_yaw: f64,
    pub footfalls: Vec<Footfall>,
}

/// An arm joint swung over one walk, about where it stood at dispatch.
#[derive(Debug, Clone)]
pub(crate) struct ArmSwing {
    pub joint: usize,
    pub center: f64,
    pub amplitude: f64,
}

/// The footfalls of every leg over one dispatch, planned at dispatch.
#[derive(Debug, Clone)]
pub(crate) struct GaitPlan {
    pub profile: BodyProfile,
    pub legs: Vec<LegPlan>,
    /// When the last foot lands — the walk (and the settle after arrival)
    /// is over, and the legs stand.
    pub done: f64,
    /// The arms this walk swings: decided at dispatch, and left alone
    /// (not even returned to a centre) when the hands are full or a ramp
    /// is driving them.
    pub swing: Vec<ArmSwing>,
    pub sway: Option<BodySway>,
}

/// What a leg is doing at an instant.
pub(crate) enum LegState {
    Planted(Isometry3<f64>),
    /// Mid-swing between two anchors.
    Swinging {
        from: Isometry3<f64>,
        to: Isometry3<f64>,
        u: f64,
    },
}

impl GaitPlan {
    /// Every DOF the walk drives: the legs, and the arms it swings.
    pub fn owned(&self, gait: &ResolvedGait) -> Vec<usize> {
        let mut out = gait.leg_joints();
        out.extend(self.swing.iter().map(|s| s.joint));
        out
    }

    /// Where the arms and legs rest once the walk is over: the stance for
    /// the legs, the dispatch-time centre for each swung arm.
    pub fn rest(&self, gait: &ResolvedGait) -> Vec<f64> {
        let mut rest = gait.stance.clone();
        for s in &self.swing {
            rest[s.joint] = s.center;
        }
        rest
    }

    /// The anchor a leg stands on (or left) at `t`, with any swing in
    /// flight: `(position, yaw, in-flight footfall)`.
    pub fn anchor(&self, leg: usize, t: f64) -> (Point3<f64>, f64, Option<&Footfall>) {
        let plan = &self.legs[leg];
        let mut prev = (plan.start, plan.start_yaw);
        for f in &plan.footfalls {
            if t < f.lift {
                return (prev.0, prev.1, None);
            }
            if t < f.land {
                return (prev.0, prev.1, Some(f));
            }
            prev = (f.position, f.yaw);
        }
        (prev.0, prev.1, None)
    }

    pub fn state(&self, gait: &ResolvedGait, leg: usize, t: f64) -> LegState {
        let (position, yaw, flying) = self.anchor(leg, t);
        let from = gait.foot_pose(leg, &position, yaw);
        match flying {
            None => LegState::Planted(from),
            Some(f) => LegState::Swinging {
                from,
                to: gait.foot_pose(leg, &f.position, f.yaw),
                u: (t - f.lift) / (f.land - f.lift),
            },
        }
    }
}

/// Plans every leg's footfalls for a drive. Raibert's symmetric placement,
/// in closed form: a foot lands where its stance will be centred under
/// the body at mid-stance, which is a half-stance ahead on a straight and
/// a step around the origin on a pivot. After the vehicle stops the frame
/// holds, so the footfalls converge on the parked positions by themselves —
/// the settle is the gait continuing until every foot stands where the
/// stance puts it, at most one more cycle per leg. `feet` are the feet's
/// world anchors (and the heading they stand at) at dispatch; `carry`
/// holds, per leg, a swing already in flight (a goto issued mid-settle),
/// which completes as planned before the new cycles begin.
pub(crate) fn plan_gait(
    gait: &ResolvedGait,
    offset: &Isometry3<f64>,
    profile: BodyProfile,
    feet: &[(Point3<f64>, f64)],
    carry: &[Option<Footfall>],
    swing: Vec<ArmSwing>,
) -> GaitPlan {
    let (t0, t_end) = (profile.t0, profile.t_end);
    let swing_time = gait.swing();
    let stance_half = 0.5 * gait.duty * gait.period;
    let mut legs = Vec::with_capacity(gait.legs.len());
    let mut done = t0;
    for (i, leg) in gait.legs.iter().enumerate() {
        let foot_at = |frame: &Isometry3<f64>| -> Point3<f64> {
            Point3::from((frame * offset * leg.nominal).translation.vector)
        };
        let parked = foot_at(&profile.end_frame);
        let mut footfalls: Vec<Footfall> = Vec::new();
        let mut last = feet[i].0;
        let mut earliest = t0;
        if let Some(f) = &carry[i] {
            last = f.position;
            earliest = f.land;
            footfalls.push(f.clone());
        }
        let mut k = 0usize;
        loop {
            let lift = t0 + (k as f64 + gait.phases[i]) * gait.period;
            k += 1;
            if lift < earliest - 1e-9 {
                continue;
            }
            if lift >= t_end - 1e-9 && (last - parked).norm() < 1e-9 {
                break;
            }
            let land = lift + swing_time;
            let mid = land + stance_half;
            let frame = profile.frame_at(mid);
            let position = foot_at(&frame);
            footfalls.push(Footfall {
                leg: leg.name.clone(),
                lift,
                land,
                position,
                yaw: yaw_of(&frame),
            });
            last = position;
            if k > 1_000_000 {
                break;
            }
        }
        if let Some(f) = footfalls.last() {
            done = done.max(f.land);
        }
        legs.push(LegPlan {
            start: feet[i].0,
            start_yaw: feet[i].1,
            footfalls,
        });
    }
    let sway = (gait.bob > 0.0 || gait.lateral > 0.0).then_some(BodySway {
        t0,
        done,
        period: gait.period,
        swing: swing_time,
        bob: gait.bob,
        lateral: gait.lateral,
        lean: gait.lean,
    });
    GaitPlan {
        profile,
        legs,
        done,
        swing,
        sway,
    }
}

/// Where a swinging foot is at progress `u ∈ [0, 1]`: eased along the
/// chord, lifted on a half-sine — at rest at both ends, so nothing jerks at
/// lift-off or touchdown — and turned from the heading it left at to the
/// one it lands with.
pub(crate) fn swing_pose(
    from: &Isometry3<f64>,
    to: &Isometry3<f64>,
    lift: f64,
    u: f64,
) -> Isometry3<f64> {
    let u = u.clamp(0.0, 1.0);
    let s = u * u * (3.0 - 2.0 * u);
    let mut p = from.translation.vector + (to.translation.vector - from.translation.vector) * s;
    p.z += lift * (PI * u).sin();
    let rotation = from
        .rotation
        .try_slerp(&to.rotation, s, 1e-9)
        .unwrap_or(to.rotation);
    Isometry3::from_parts(Translation3::from(p), rotation)
}

/// World foot positions of every leg at `q` with the root at `base`, with
/// the body's heading.
pub(crate) fn feet_at(
    model: &RobotModel,
    gait: &ResolvedGait,
    q: &[f64],
    base: &Isometry3<f64>,
) -> Vec<(Point3<f64>, f64)> {
    let poses = botrail_kin::forward_kinematics_with_base(model, q, base).expect("q has robot DOF");
    let yaw = yaw_of(base);
    gait.legs
        .iter()
        .map(|l| (Point3::from(poses[l.foot].translation.vector), yaw))
        .collect()
}
