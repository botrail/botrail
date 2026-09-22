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

/// How far above or below the vehicle plane a walkable surface is searched
/// for a foothold when the gait declares no `max_step`: covers industrial
/// stair risers (180–250 mm) with room to spare.
pub(crate) const DEFAULT_STEP_REACH: f64 = 0.3;

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

/// The body's pitch over one walk. A machine on a grade tilts onto it —
/// its hips follow the ground, so every leg works the range it works on the
/// flat. Without it a level body on a stair asks the downhill legs to reach
/// half a machine-length of grade *plus* half a riser, which is more than a
/// real leg has. Piecewise and closed form: the angle holds over each leg of
/// the drive and blends over one body length where the grade changes, so any
/// resample rate sees the same body.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyPitch {
    pub t0: f64,
    pub t1: f64,
    /// Nose-up angle at each end, radians.
    pub from: f64,
    pub to: f64,
}

/// How high the body rides over its guide line at one moment. A machine
/// on stairs does not hold one height above the straight line its route
/// draws — it rides the steps: as the feet climb, the body climbs with
/// them. Without it a fixed stance has to serve both extremes at once (a
/// leg reaching down to a low tread while another is at its swing apex),
/// which costs half a riser more of leg travel than a real machine spends
/// and caps the flight far below what the machine is rated for.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyRise {
    pub t0: f64,
    pub t1: f64,
    /// Height over the guide plane at each end, metres.
    pub from: f64,
    pub to: f64,
}

/// The rise at `t`: held before the first span and after the last.
pub fn rise_at(rises: &[BodyRise], t: f64) -> f64 {
    let Some(first) = rises.first() else {
        return 0.0;
    };
    if t <= first.t0 {
        return first.from;
    }
    for span in rises {
        if t < span.t1 {
            let width = span.t1 - span.t0;
            if width <= 1e-12 || t <= span.t0 {
                return span.from;
            }
            return span.from + (span.to - span.from) * smooth((t - span.t0) / width);
        }
    }
    rises.last().map(|s| s.to).unwrap_or(0.0)
}

/// Where one leg's foot is at `t`: its anchor, gliding along the swing
/// chord while it flies (so the mean over the legs moves, never steps);
/// riding, it blends from where the walk left it to the stance height over
/// the guide line.
fn foot_height(
    plan: &LegPlan,
    rolls: &[RollRegion],
    profile: &BodyProfile,
    foot_radius: f64,
    t: f64,
) -> f64 {
    let guide = |t: f64| profile.frame_at(t).translation.z + foot_radius;
    let riding = |from: f64, since: f64, region: Option<&RollRegion>| -> f64 {
        let blend = region.map(|r| r.blend).unwrap_or(0.0);
        if blend <= 1e-12 {
            return guide(t);
        }
        from + (guide(t) - from) * smooth((t - since) / blend)
    };
    let mut prev_z: Option<f64> = None;
    for (w, walk) in plan.walks.iter().enumerate() {
        let region_before = rolls.iter().find(|r| r.next_window == Some(w));
        if let Some(region) = region_before {
            if t < region.t1 {
                // Riding into this window from the previous walk's last
                // anchor (or the dispatch anchor).
                let from = prev_z.unwrap_or(plan.dispatch.0.z);
                return riding(from, region.t0, Some(region));
            }
        }
        if t < walk.ride {
            let mut prev = walk.start.z;
            for f in &walk.footfalls {
                if t < f.lift {
                    return prev;
                }
                if t < f.land {
                    let u = (t - f.lift) / (f.land - f.lift).max(1e-12);
                    return prev + (f.position.z - prev) * smooth(u);
                }
                prev = f.position.z;
            }
            return prev;
        }
        prev_z = Some(
            walk.footfalls
                .last()
                .map(|f| f.position.z)
                .unwrap_or(walk.start.z),
        );
        if let Some(region) = rolls.iter().find(|r| r.next_window == Some(w + 1)) {
            if t < region.t1 {
                return riding(prev_z.expect("set"), walk.ride, Some(region));
            }
        }
    }
    // Riding out the route after the last window (or, with no window at
    // all, from the dispatch).
    let last = plan.walks.last();
    let from = prev_z
        .or(last.map(|w| w.start.z))
        .unwrap_or(plan.dispatch.0.z);
    let since = last.map(|w| w.ride).unwrap_or(profile.t0);
    riding(from, since, rolls.iter().find(|r| r.next_window.is_none()))
}

/// How the body rides a drive: the mean of where its feet are, over the
/// guide plane, sampled at every lift and landing (between them the legs
/// glide, so this is the curve itself, not a fit to it) — and at the ends
/// of every blend into a roll, where the legs return to the stance.
pub(crate) fn plan_rise(
    legs: &[LegPlan],
    rolls: &[RollRegion],
    profile: &BodyProfile,
    foot_radius: f64,
) -> Vec<BodyRise> {
    if legs.is_empty() {
        return Vec::new();
    }
    let mut times: Vec<f64> = vec![profile.t0];
    for leg in legs {
        for walk in &leg.walks {
            for f in &walk.footfalls {
                times.push(f.lift);
                times.push(f.land);
            }
            if walk.ride.is_finite() {
                times.push(walk.ride);
                if let Some(r) = rolls
                    .iter()
                    .find(|r| r.t0 <= walk.ride && walk.ride <= r.t1)
                {
                    times.push((walk.ride + r.blend).min(r.t1));
                }
            }
        }
    }
    for r in rolls {
        times.push(r.t0);
        times.push((r.t0 + r.blend).min(r.t1));
        times.push(r.t1);
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    times.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    let value = |t: f64| -> f64 {
        let mean: f64 = legs
            .iter()
            .map(|l| foot_height(l, rolls, profile, foot_radius, t))
            .sum::<f64>()
            / legs.len() as f64;
        mean - foot_radius - profile.frame_at(t).translation.z
    };
    let mut spans: Vec<BodyRise> = Vec::new();
    for pair in times.windows(2) {
        let (t0, t1) = (pair[0], pair[1]);
        let (from, to) = (value(t0), value(t1));
        if t1 - t0 < 1e-12 {
            continue;
        }
        // A level stretch extends a level span before it — never a blend,
        // which would stretch the blend out over the whole stretch.
        if (to - from).abs() < 1e-12
            && spans
                .last()
                .is_some_and(|s| (s.to - from).abs() < 1e-12 && (s.to - s.from).abs() < 1e-12)
        {
            spans.last_mut().expect("checked").t1 = t1;
            continue;
        }
        spans.push(BodyRise { t0, t1, from, to });
    }
    spans
}

/// The pitch angle at `t`: held before the first span and after the last
/// (a machine parked on a grade stands on it).
pub fn pitch_angle(pitches: &[BodyPitch], t: f64) -> f64 {
    let Some(first) = pitches.first() else {
        return 0.0;
    };
    if t <= first.t0 {
        return first.from;
    }
    for span in pitches {
        if t < span.t1 {
            let width = span.t1 - span.t0;
            if width <= 1e-12 || t <= span.t0 {
                return span.from;
            }
            return span.from + (span.to - span.from) * smooth((t - span.t0) / width);
        }
    }
    pitches.last().map(|s| s.to).unwrap_or(0.0)
}

/// The body-frame offset that tilts a walking body onto its grade.
pub fn pitch_offset(pitches: &[BodyPitch], t: f64) -> Isometry3<f64> {
    let angle = pitch_angle(pitches, t);
    if angle.abs() < 1e-12 {
        return Isometry3::identity();
    }
    Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -angle),
    )
}

/// The pitch a drive asks of a body `body_len` long: the grade of each
/// piece, blended over the time the machine takes to cover its own length.
pub(crate) fn plan_pitch(profile: &BodyProfile, body_len: f64) -> Vec<BodyPitch> {
    let mut knots: Vec<(f64, f64)> = vec![(profile.t0, 0.0)];
    let mut grade = 0.0;
    for (a, b, _frame, piece) in &profile.pieces {
        let (g, speed) = match piece {
            VehiclePiece::Lin { velocity } => {
                let run = velocity.x.hypot(velocity.y);
                if run > 1e-9 {
                    (velocity.z / run, run)
                } else {
                    (grade, 0.0)
                }
            }
            // A pivot turns on whatever it stands on.
            VehiclePiece::Piv { .. } => (grade, 0.0),
        };
        grade = g;
        let half = 0.5 * (b - a);
        let blend = if speed > 1e-9 {
            (0.5 * body_len / speed).min(half)
        } else {
            half
        };
        knots.push((a + blend, g.atan()));
        knots.push((b - blend, g.atan()));
    }
    // The drive ends level: the settle puts the legs back in the stance,
    // and the stance is what a parked machine stands in.
    knots.push((profile.t_end, 0.0));
    let mut spans: Vec<BodyPitch> = Vec::new();
    for pair in knots.windows(2) {
        let ((t0, from), (t1, to)) = (pair[0], pair[1]);
        if t1 - t0 < 1e-12 {
            continue;
        }
        // A held angle extends a held span before it — never a blend,
        // which would stretch the blend out over the rest of the drive.
        if (to - from).abs() < 1e-12
            && spans
                .last()
                .is_some_and(|s| (s.to - from).abs() < 1e-12 && (s.to - s.from).abs() < 1e-12)
        {
            spans.last_mut().expect("checked").t1 = t1;
            continue;
        }
        spans.push(BodyPitch { t0, t1, from, to });
    }
    spans
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
    /// Foot radius (see `GaitSpec::foot_radius`).
    pub foot_radius: f64,
    /// The margin a foothold needs from a tread's edge (see
    /// `GaitSpec::foothold`).
    pub foothold: f64,
    /// Declared step ability (see `GaitSpec::max_step`).
    pub max_step: Option<f64>,
    /// The link the legs hang from — the machine's body. What a walking
    /// vehicle carries rides *this*, not the line its route draws.
    pub body: usize,
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
        joint_mask: None,
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
        ("foothold", spec.foothold.unwrap_or(spec.foot_radius)),
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
        foot_radius: spec.foot_radius,
        foothold: spec.foothold.unwrap_or(spec.foot_radius),
        max_step: spec.max_step,
        body,
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
    /// Which gear takes each piece, indexed like `pieces` — a wheel-legged
    /// machine's route (design-wheel-legged.md §3.3). Empty for a machine
    /// that walks every piece.
    pub modes: Vec<LegMode>,
}

/// Which running gear one piece of a route is taken on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LegMode {
    Roll,
    Walk,
}

/// How a mounted machine took one stretch of its route: the gear, the
/// interval, and how far it went. A walking machine's spans all walk, a
/// rolling one's all roll; a wheel-legged one's say where it changed.
#[derive(Debug, Clone, PartialEq)]
pub struct LocomotionSpan {
    pub t0: f64,
    pub t1: f64,
    pub mode: LegMode,
    /// Metres travelled over the span (a pivot travels none).
    pub distance: f64,
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

    /// The mode of piece `i`: walked, when the profile states none.
    pub fn mode(&self, i: usize) -> LegMode {
        self.modes.get(i).copied().unwrap_or(LegMode::Walk)
    }

    /// The stretches taken on `mode`, consecutive pieces merged: `[a, b]`
    /// in time order.
    pub fn windows(&self, mode: LegMode) -> Vec<(f64, f64)> {
        let mut out: Vec<(f64, f64)> = Vec::new();
        for (i, (a, b, _, _)) in self.pieces.iter().enumerate() {
            if self.mode(i) != mode {
                continue;
            }
            match out.last_mut() {
                Some(last) if (last.1 - a).abs() < 1e-9 => last.1 = *b,
                _ => out.push((*a, *b)),
            }
        }
        out
    }

    /// The spans a track records for this drive.
    pub fn locomotion(&self) -> Vec<LocomotionSpan> {
        let mut out: Vec<LocomotionSpan> = Vec::new();
        for (i, (a, b, _, piece)) in self.pieces.iter().enumerate() {
            let mode = self.mode(i);
            let distance = match piece {
                VehiclePiece::Lin { velocity } => velocity.norm() * (b - a),
                VehiclePiece::Piv { .. } => 0.0,
            };
            match out.last_mut() {
                Some(last) if last.mode == mode && (last.t1 - a).abs() < 1e-9 => {
                    last.t1 = *b;
                    last.distance += distance;
                }
                _ => out.push(LocomotionSpan {
                    t0: *a,
                    t1: *b,
                    mode,
                    distance,
                }),
            }
        }
        out
    }
}

/// Heading of a frame about +Z.
pub(crate) fn yaw_of(frame: &Isometry3<f64>) -> f64 {
    let r = frame.rotation.to_rotation_matrix();
    r[(1, 0)].atan2(r[(0, 0)])
}

/// One leg's footfalls over one walk window of a dispatch.
#[derive(Debug, Clone)]
pub(crate) struct LegWalk {
    /// Where the foot stood as the window opened, and the body heading it
    /// stood at.
    pub start: Point3<f64>,
    pub start_yaw: f64,
    pub footfalls: Vec<Footfall>,
    /// When the leg stops walking after the window and rides the body
    /// instead: the window's end, or the landing of a swing still in the
    /// air then. Infinite when the window ends the route — the legs settle
    /// into the stance and stand.
    pub ride: f64,
}

/// One leg's planned footfalls over a dispatch: one walk per walk window.
#[derive(Debug, Clone)]
pub(crate) struct LegPlan {
    /// Where the foot stood at dispatch, and the heading it stood at —
    /// what it rides from when the route starts rolling.
    pub dispatch: (Point3<f64>, f64),
    pub walks: Vec<LegWalk>,
}

/// A stretch of the route the machine rolls — before the first walk
/// window, or after any window that does not end the route. The legs ride
/// the body through it, blending from where the walk left them to the
/// stance over `blend` seconds.
#[derive(Debug, Clone)]
pub(crate) struct RollRegion {
    pub t0: f64,
    pub t1: f64,
    pub blend: f64,
    /// The walk window this region runs into, if any.
    pub next_window: Option<usize>,
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
    /// The walk windows `[a, b]` of the route, in time order: the whole
    /// drive for a machine that only walks.
    pub windows: Vec<(f64, f64)>,
    /// The roll regions between and around them (empty for a machine that
    /// only walks).
    pub rolls: Vec<RollRegion>,
    pub legs: Vec<LegPlan>,
    /// When the last foot lands — the walk (and the settle after arrival)
    /// is over, and the legs stand. On a route that ends rolling, when the
    /// last leg has blended into the stance.
    pub done: f64,
    /// The arms this walk swings: decided at dispatch, and left alone
    /// (not even returned to a centre) when the hands are full or a ramp
    /// is driving them.
    pub swing: Vec<ArmSwing>,
    pub sway: Option<BodySway>,
    /// How the body tilts over this drive (empty on the level).
    pub pitch: Vec<BodyPitch>,
    /// How the body rides over the guide line (empty on flat ground).
    pub rise: Vec<BodyRise>,
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
    /// Riding the body through a roll region: from the anchor pose it
    /// left the walk at, blending to the stance.
    Riding {
        region: usize,
        from: Isometry3<f64>,
    },
}

/// The longest a leg takes to blend into the stance once it rides — and
/// what a roll has to have room for twice over, on top of a swing, to be
/// worth changing gear for.
pub(crate) const BLEND_MAX: f64 = 1.0;

impl GaitPlan {
    /// The DOF the walk owns at `t`: the legs until the last foot settles,
    /// the swung arms only until the vehicle stops — from then on a move
    /// may claim an arm (the reach a walk ends in), and the walk leaves
    /// it alone.
    pub fn owned_at(&self, gait: &ResolvedGait, t: f64) -> Vec<usize> {
        let mut out = gait.leg_joints();
        if t < self.profile.t_end - 1e-9 {
            out.extend(self.swing.iter().map(|s| s.joint));
        }
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

    /// The roll region a leg rides before window `w` (`None`: the last
    /// region, after every window).
    fn region_before(&self, w: Option<usize>) -> usize {
        self.rolls
            .iter()
            .position(|r| r.next_window == w)
            .expect("every roll region is planned")
    }

    /// The anchor a leg stands on (or left) at `t`, with any swing in
    /// flight: `(position, yaw, in-flight footfall)` — `None` while the
    /// leg rides the body through a roll region.
    pub fn anchor(&self, leg: usize, t: f64) -> Option<(Point3<f64>, f64, Option<&Footfall>)> {
        let plan = &self.legs[leg];
        for (w, (a, _)) in self.windows.iter().enumerate() {
            if t < *a {
                return None;
            }
            let walk = &plan.walks[w];
            if t >= walk.ride {
                continue;
            }
            let mut prev = (walk.start, walk.start_yaw);
            for f in &walk.footfalls {
                if t < f.lift {
                    return Some((prev.0, prev.1, None));
                }
                if t < f.land {
                    return Some((prev.0, prev.1, Some(f)));
                }
                prev = (f.position, f.yaw);
            }
            return Some((prev.0, prev.1, None));
        }
        None
    }

    pub fn state(&self, gait: &ResolvedGait, leg: usize, t: f64) -> LegState {
        let plan = &self.legs[leg];
        for (w, (a, _)) in self.windows.iter().enumerate() {
            let walk = &plan.walks[w];
            if t < *a {
                // Riding into this window: from where the previous walk
                // (or the dispatch) left the foot.
                let from = match w.checked_sub(1) {
                    Some(p) => plan.walks[p].last_anchor(),
                    None => plan.dispatch,
                };
                return LegState::Riding {
                    region: self.region_before(Some(w)),
                    from: gait.foot_pose(leg, &from.0, from.1),
                };
            }
            if t >= walk.ride {
                continue;
            }
            let mut prev = (walk.start, walk.start_yaw);
            for f in &walk.footfalls {
                if t < f.lift {
                    return LegState::Planted(gait.foot_pose(leg, &prev.0, prev.1));
                }
                if t < f.land {
                    return LegState::Swinging {
                        from: gait.foot_pose(leg, &prev.0, prev.1),
                        to: gait.foot_pose(leg, &f.position, f.yaw),
                        u: (t - f.lift) / (f.land - f.lift),
                    };
                }
                prev = (f.position, f.yaw);
            }
            return LegState::Planted(gait.foot_pose(leg, &prev.0, prev.1));
        }
        // Past every window: riding out the route — from where the last
        // walk left the foot, or (a route rolled throughout, dispatched
        // while the legs were still settling) from where it stood.
        let from = plan
            .walks
            .last()
            .map(|w| w.last_anchor())
            .unwrap_or(plan.dispatch);
        LegState::Riding {
            region: self.region_before(None),
            from: gait.foot_pose(leg, &from.0, from.1),
        }
    }
}

impl LegWalk {
    /// Where the foot stands when this walk is over.
    fn last_anchor(&self) -> (Point3<f64>, f64) {
        self.footfalls
            .last()
            .map(|f| (f.position, f.yaw))
            .unwrap_or((self.start, self.start_yaw))
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
    // `floor(x, y, hint)`: the walkable surface under a foothold at
    // `(x, y)`, if any — a stair tread instead of the slope the guide line
    // interpolates. `hint` is the surface the foot stands on *now*: a step
    // is measured from the foot, not from the body, which is half a machine
    // away and, on a pitch, a step lower. `None` leaves the foot on the
    // guide surface the tilted stance puts it on.
    floor: &dyn Fn(f64, f64, f64) -> Option<f64>,
) -> GaitPlan {
    let (t0, t_end) = (profile.t0, profile.t_end);
    // The body tilts onto the grade; the footholds are read from the tilted
    // stance, so a leg reaches the same way uphill as it does on the flat.
    let (front, back) = gait.legs.iter().fold((f64::MIN, f64::MAX), |(hi, lo), l| {
        let x = l.nominal.translation.x;
        (hi.max(x), lo.min(x))
    });
    let pitch = plan_pitch(&profile, (front - back).abs().max(1e-6));
    let swing_time = gait.swing();
    let stance_half = 0.5 * gait.duty * gait.period;
    // The walk windows of the route, and the roll regions around them. A
    // route rolled throughout (dispatched while a walk was still settling,
    // or the legs would not be planned at all) gets an empty window at the
    // dispatch: a swing in the air lands as planned, and the legs ride
    // from there.
    let mut windows = profile.windows(LegMode::Walk);
    if windows.is_empty() {
        windows.push((t0, t0));
    }
    let mut rolls: Vec<RollRegion> = Vec::new();
    let region = |from: f64, to: f64, after_walk: bool, next: Option<usize>| RollRegion {
        t0: from,
        t1: to,
        // A swing still in the air at the window's end lands first.
        blend: (0.5 * (to - from - if after_walk { swing_time } else { 0.0 }).max(0.0))
            .min(BLEND_MAX),
        next_window: next,
    };
    match windows.first() {
        Some((a, _)) if *a > t0 + 1e-9 => rolls.push(region(t0, *a, false, Some(0))),
        None => rolls.push(region(t0, t_end, false, None)),
        _ => {}
    }
    for (w, (_, b)) in windows.iter().enumerate() {
        if *b < t_end - 1e-9 {
            let next = windows.get(w + 1).map(|n| n.0).unwrap_or(t_end);
            rolls.push(region(
                *b,
                next,
                true,
                (w + 1 < windows.len()).then_some(w + 1),
            ));
        }
    }
    let mut legs = Vec::with_capacity(gait.legs.len());
    let mut done = t0;
    for (i, leg) in gait.legs.iter().enumerate() {
        let foot_at = |frame: &Isometry3<f64>, from: &Point3<f64>, tilt: f64| -> Point3<f64> {
            let lean = pitch_offset(
                &[BodyPitch {
                    t0: 0.0,
                    t1: 0.0,
                    from: tilt,
                    to: tilt,
                }],
                0.0,
            );
            let mut p = Point3::from((frame * offset * lean * leg.nominal).translation.vector);
            // The foot stands `foot_radius` above whatever it stands on, so
            // the surface it is on now is what the next one is measured from.
            let hint = from.z - gait.foot_radius;
            if let Some(surface) = floor(p.x, p.y, hint) {
                // On real ground the foot sits on it exactly — the tilt
                // decided *where* the foot reaches, not how high it floats.
                p.z = surface + gait.foot_radius;
            }
            p
        };
        let mut walks = Vec::with_capacity(windows.len());
        for (w, (a, b)) in windows.iter().enumerate() {
            let final_window = *b >= t_end - 1e-9;
            // Where the foot stands as the window opens: at dispatch,
            // wherever it is; after a roll, at the stance under the body.
            let (start, start_yaw) = if w == 0 && *a <= t0 + 1e-9 {
                feet[i]
            } else {
                let frame = profile.frame_at(*a);
                let lean = pitch_offset(
                    &[BodyPitch {
                        t0: 0.0,
                        t1: 0.0,
                        from: pitch_angle(&pitch, *a),
                        to: pitch_angle(&pitch, *a),
                    }],
                    0.0,
                );
                (
                    Point3::from((frame * offset * lean * leg.nominal).translation.vector),
                    yaw_of(&frame),
                )
            };
            let mut footfalls: Vec<Footfall> = Vec::new();
            let mut last = start;
            let mut earliest = *a;
            if w == 0 {
                if let Some(f) = &carry[i] {
                    let mut f = f.clone();
                    // Into a roll, the swing in the air was planned for a
                    // body that stayed put; it lands under its hip instead,
                    // where the ride takes it from.
                    if *b <= *a + 1e-9 {
                        let frame = profile.frame_at(f.land);
                        f.position = foot_at(&frame, &start, pitch_angle(&pitch, f.land));
                        f.yaw = yaw_of(&frame);
                    }
                    last = f.position;
                    earliest = f.land;
                    footfalls.push(f);
                }
            }
            let mut k = 0usize;
            loop {
                let lift = a + (k as f64 + gait.phases[i]) * gait.period;
                k += 1;
                if lift < earliest - 1e-9 {
                    continue;
                }
                if final_window {
                    // Where the walk leaves this foot: re-read as the foot
                    // climbs, since the ground it parks on is found from
                    // where it stands.
                    let parked = foot_at(&profile.end_frame, &last, pitch_angle(&pitch, t_end));
                    if lift >= t_end - 1e-9 && (last - parked).norm() < 1e-9 {
                        break;
                    }
                } else if lift >= *b - 1e-9 {
                    break;
                }
                let land = lift + swing_time;
                let mid = land + stance_half;
                // A foot still in the air when the window closes lands
                // under its hip, where the ride takes it from; one landing
                // inside the window is placed for the body no later than
                // the window's end, where the walk's pace ends.
                let place = if final_window {
                    mid
                } else if land > *b {
                    land
                } else {
                    mid.min(*b)
                };
                let frame = profile.frame_at(place);
                let position = foot_at(&frame, &last, pitch_angle(&pitch, place));
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
            let ride = if final_window {
                f64::INFINITY
            } else {
                footfalls.last().map(|f| f.land.max(*b)).unwrap_or(*b)
            };
            if let Some(f) = footfalls.last() {
                done = done.max(f.land);
            }
            if ride.is_finite() {
                let blend = rolls
                    .iter()
                    .find(|r| (r.t0 - b).abs() < 1e-9)
                    .map(|r| r.blend)
                    .unwrap_or(0.0);
                done = done.max(ride + blend);
            }
            walks.push(LegWalk {
                start,
                start_yaw,
                footfalls,
                ride,
            });
        }
        legs.push(LegPlan {
            dispatch: feet[i],
            walks,
        });
    }
    if let Some(first) = rolls.first().filter(|r| r.t0 <= t0 + 1e-9) {
        done = done.max(first.t0 + first.blend);
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
    let rise = plan_rise(&legs, &rolls, &profile, gait.foot_radius);
    GaitPlan {
        profile,
        windows,
        rolls,
        legs,
        done,
        swing,
        sway,
        pitch,
        rise,
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
