//! Driving a cloth cell from a baked cycle.
//!
//! The cloth never drives a robot, so it is simulated after the bake: the
//! pass reads each gripper's tool pose off the robot tracks and whether it
//! is closed off a signal, steps a [`ClothCell`] through the cycle on the
//! cloth's own clock, and returns the vertex track the timeline carries.

use crate::{ClothCell, ClothCellError, ClothSpec, ClothTrack, Pad};
use botrail_scene::rollout::SequenceTimeline;
use botrail_scene::Scene;
use nalgebra::Isometry3;

/// What a pinned grasp softens from before it opens (m/N): ten times this
/// on the first softened step.
const HARD_SOFTEN_BASE: f64 = 0.002;

/// How many times a step the solver rejects is halved before the pass
/// gives up: down to eighths of a step.
const MAX_SPLITS: u32 = 3;

/// How a gripper takes and lets go of cloth.
#[derive(Debug, Clone, PartialEq)]
pub struct Grip {
    /// Closing takes the free vertices within this distance of the tool
    /// point (m), through every layer. A patch another gripper holds there
    /// is handed over whole instead.
    pub radius: f64,
    /// 0 pins the patch to the gripper; a positive compliance (m/N) holds
    /// it with springs of stiffness `1 / compliance`.
    pub compliance: f64,
    /// Cloth steps over which the hold softens, tenfold per step, before it
    /// opens (0 opens at once). A taut flap then relaxes onto what is under
    /// it instead of snapping free.
    pub soften_steps: usize,
    /// A box the gripper pushes cloth with; `None` passes through cloth it
    /// does not hold.
    pub pad: Option<Pad>,
}

impl Default for Grip {
    fn default() -> Self {
        Grip {
            radius: 0.04,
            compliance: 0.0,
            soften_steps: 0,
            pad: None,
        }
    }
}

/// One gripper of a pass: the robot link whose frame is the tool point, and
/// the signal that closes it.
#[derive(Debug, Clone, PartialEq)]
pub struct GripperBinding {
    /// Scene index of the robot.
    pub robot: usize,
    /// Link index on that robot.
    pub link: usize,
    /// The gripper holds while this signal is true.
    pub signal: String,
    pub grip: Grip,
}

/// A cloth and the grippers that handle it, to simulate against a cycle.
#[derive(Debug, Clone)]
pub struct ClothPass {
    /// Name of the track.
    pub name: String,
    pub spec: ClothSpec,
    /// Height of the level support surface the cloth lies on.
    pub table_top: Option<f64>,
    pub grippers: Vec<GripperBinding>,
    /// Cloth step (s); 0.1 is the solver's qualified step. The cycle is
    /// divided into equal steps of about this length.
    pub step: f64,
}

/// A gripper as the pass sees it: where its tool point is and whether it is
/// closed, at any time of the cycle.
pub struct GripperDrive<'a> {
    pub pose: &'a dyn Fn(f64) -> Isometry3<f64>,
    pub closed: &'a dyn Fn(f64) -> bool,
    pub grip: Grip,
}

/// Steps a cloth through `duration` seconds under `grippers`.
///
/// The grippers' state is read at the cloth's sample times: one that is
/// closed at a sample and was open at the one before takes the vertices
/// within its radius (or takes over the patch another gripper holds
/// there), and one that is open after being closed lets go. A pulse shorter
/// than a step between two samples is not seen.
///
/// A step the solver rejects is retried in halves, the grippers stopping
/// half-way, and a rejected half again, down to eighths of a step. If an
/// eighth is rejected too the pass stops there, and the track says when and
/// why ([`ClothTrack::failure`]) and holds its last state.
pub fn drive(
    name: &str,
    spec: &ClothSpec,
    table_top: Option<f64>,
    grippers: &[GripperDrive<'_>],
    duration: f64,
    step: f64,
) -> Result<ClothTrack, ClothCellError> {
    if !(step.is_finite() && step > 0.0) {
        return Err(ClothCellError::Pass(
            "the cloth step must be positive".into(),
        ));
    }
    if !(duration.is_finite() && duration >= 0.0) {
        return Err(ClothCellError::Pass(
            "the cycle has no finite duration".into(),
        ));
    }
    for (i, g) in grippers.iter().enumerate() {
        if !(g.grip.radius.is_finite() && g.grip.radius > 0.0) {
            return Err(ClothCellError::Pass(format!(
                "gripper {i}: the grasp radius must be positive"
            )));
        }
        if !(g.grip.compliance.is_finite() && g.grip.compliance >= 0.0) {
            return Err(ClothCellError::Pass(format!(
                "gripper {i}: the compliance must not be negative"
            )));
        }
    }
    // Equal steps that land on the end of the cycle.
    let n = if duration > 0.0 {
        (duration / step).round().max(1.0) as usize
    } else {
        0
    };
    let time = |k: usize| {
        if k == n {
            duration
        } else {
            duration * k as f64 / n as f64
        }
    };
    let pads: Vec<Option<Pad>> = grippers.iter().map(|g| g.grip.pad).collect();
    let mut cell = ClothCell::new(spec, table_top, &pads)?;
    let closed: Vec<Vec<bool>> = grippers
        .iter()
        .map(|g| (0..=n).map(|k| (g.closed)(time(k))).collect())
        .collect();
    let mut track = ClothTrack {
        name: name.to_string(),
        triangles: cell.triangles().to_vec(),
        landmarks: cell.landmarks().clone(),
        ..ClothTrack::default()
    };
    for (i, g) in grippers.iter().enumerate() {
        cell.place_gripper(i, &(g.pose)(0.0))?;
    }
    events(&mut cell, grippers, &closed, 0, 0.0, &mut track.warnings)?;
    record(&cell, &mut track, 0.0);
    for k in 1..=n {
        let (t0, t1) = (time(k - 1), time(k));
        soften(&mut cell, grippers, &closed, k)?;
        let samples = track.times.len();
        match advance_split(&mut cell, grippers, &mut track, t0, t1, 0) {
            Ok(()) => {
                let parts = track.times.len() - samples + 1;
                if parts > 1 {
                    track
                        .warnings
                        .push(format!("the step to {t1:.2} s was taken in {parts} parts"));
                }
            }
            Err(ClothCellError::Rejected(reason)) => {
                let reached = track.times.last().copied().unwrap_or(0.0);
                track.failure = Some((reached, reason));
                break;
            }
            Err(e) => return Err(e),
        }
        events(&mut cell, grippers, &closed, k, t1, &mut track.warnings)?;
        record(&cell, &mut track, t1);
    }
    track.fold_holds();
    Ok(track)
}

/// Moves every gripper to where it is at `to` and steps the cloth there.
fn advance(
    cell: &mut ClothCell,
    grippers: &[GripperDrive<'_>],
    from: f64,
    to: f64,
) -> Result<(), ClothCellError> {
    for (i, g) in grippers.iter().enumerate() {
        cell.set_gripper_pose(i, &(g.pose)(to))?;
    }
    cell.step(to - from).map(|_| ())
}

/// [`advance`], halving a step the solver rejects (and a rejected half
/// again, [`MAX_SPLITS`] deep) and recording the state reached in between.
fn advance_split(
    cell: &mut ClothCell,
    grippers: &[GripperDrive<'_>],
    track: &mut ClothTrack,
    from: f64,
    to: f64,
    depth: u32,
) -> Result<(), ClothCellError> {
    match advance(cell, grippers, from, to) {
        Err(ClothCellError::Rejected(_)) if depth < MAX_SPLITS => {
            let mid = 0.5 * (from + to);
            advance_split(cell, grippers, track, from, mid, depth + 1)?;
            record(cell, track, mid);
            advance_split(cell, grippers, track, mid, to, depth + 1)
        }
        outcome => outcome,
    }
}

/// Softens the grasps that open within their ramp, before the step to
/// sample `k`.
fn soften(
    cell: &mut ClothCell,
    grippers: &[GripperDrive<'_>],
    closed: &[Vec<bool>],
    k: usize,
) -> Result<(), ClothCellError> {
    for (i, g) in grippers.iter().enumerate() {
        let steps = g.grip.soften_steps;
        if steps == 0 || cell.held(i).is_none() {
            continue;
        }
        // The sample at which this gripper opens next, if it does.
        let Some(release) = (k..closed[i].len()).find(|&j| !closed[i][j]) else {
            continue;
        };
        let remaining = release - k;
        if remaining < steps {
            let base = if g.grip.compliance > 0.0 {
                g.grip.compliance
            } else {
                HARD_SOFTEN_BASE
            };
            cell.soften_grasp(i, base * 10f64.powi((steps - remaining) as i32))?;
        }
    }
    Ok(())
}

/// Applies what the grippers do at sample `k`: those that closed take
/// cloth, then those that opened let go — in that order, so a hand-over at
/// one sample never drops the patch.
fn events(
    cell: &mut ClothCell,
    grippers: &[GripperDrive<'_>],
    closed: &[Vec<bool>],
    k: usize,
    t: f64,
    warnings: &mut Vec<String>,
) -> Result<(), ClothCellError> {
    for (i, g) in grippers.iter().enumerate() {
        if !(closed[i][k] && (k == 0 || !closed[i][k - 1])) {
            continue;
        }
        let near = cell.near(i, g.grip.radius)?;
        let giver = near
            .iter()
            .find_map(|&v| cell.holder_of(v))
            .filter(|&giver| giver != i);
        if let Some(giver) = giver {
            cell.transfer(giver, i)?;
            cell.soften_grasp(i, g.grip.compliance)?;
        } else if cell.grasp_near(i, g.grip.radius, g.grip.compliance)? == 0 {
            warnings.push(format!("gripper {i} closed on nothing at {t:.2} s"));
        }
    }
    for (i, lane) in closed.iter().enumerate() {
        let opened = k > 0 && lane[k - 1] && !lane[k];
        if opened && cell.held(i).is_some() {
            cell.release(i)?;
        }
    }
    Ok(())
}

fn record(cell: &ClothCell, track: &mut ClothTrack, t: f64) {
    track.times.push(t);
    track.points.push(
        cell.positions()
            .iter()
            .map(|p| p.map(|c| c as f32))
            .collect(),
    );
    track.held.push(cell.held_vertices());
}

/// Simulates a cloth against a baked cycle: each gripper's tool point
/// follows its robot link through the cycle (forward kinematics off the
/// robot's track, on its moving base if it rides one) and closes while its
/// signal is true. See [`drive`] for the event and failure rules.
pub fn animate(
    scene: &Scene,
    timeline: &SequenceTimeline,
    pass: &ClothPass,
) -> Result<ClothTrack, ClothCellError> {
    let robots = scene.robots();
    if timeline.robots.len() != robots.len() {
        return Err(ClothCellError::Pass(format!(
            "the cycle has {} robot tracks but the scene {} robots",
            timeline.robots.len(),
            robots.len()
        )));
    }
    let mut signals = Vec::with_capacity(pass.grippers.len());
    for (i, g) in pass.grippers.iter().enumerate() {
        let robot = robots.get(g.robot).ok_or_else(|| {
            ClothCellError::Pass(format!("gripper {i}: the scene has no robot {}", g.robot))
        })?;
        if g.link >= robot.model.links.len() {
            return Err(ClothCellError::Pass(format!(
                "gripper {i}: robot `{}` has no link {}",
                robot.name, g.link
            )));
        }
        if timeline.robots[g.robot].trajectory.sample(0.0).len() != robot.model.dof() {
            return Err(ClothCellError::Pass(format!(
                "gripper {i}: the cycle's track of robot `{}` does not match its model",
                robot.name
            )));
        }
        let signal = timeline
            .signals
            .iter()
            .find(|s| s.name == g.signal)
            .ok_or_else(|| {
                ClothCellError::Pass(format!(
                    "gripper {i}: the cycle has no signal `{}`",
                    g.signal
                ))
            })?;
        signals.push(signal);
    }
    let tool = |g: &GripperBinding, t: f64| -> Isometry3<f64> {
        let track = &timeline.robots[g.robot];
        let base = SequenceTimeline::base_pose(track, t).unwrap_or(*robots[g.robot].base_pose());
        botrail_kin::forward_kinematics_with_base(
            &robots[g.robot].model,
            &track.trajectory.sample(t),
            &base,
        )
        .expect("the track matches the model")[g.link]
    };
    let tool = &tool;
    let poses: Vec<Box<dyn Fn(f64) -> Isometry3<f64> + '_>> = pass
        .grippers
        .iter()
        .map(|g| Box::new(move |t: f64| tool(g, t)) as Box<dyn Fn(f64) -> Isometry3<f64> + '_>)
        .collect();
    let closed: Vec<Box<dyn Fn(f64) -> bool + '_>> = signals
        .iter()
        .map(|s| Box::new(move |t: f64| s.value_at(t)) as Box<dyn Fn(f64) -> bool + '_>)
        .collect();
    let drives: Vec<GripperDrive<'_>> = pass
        .grippers
        .iter()
        .zip(&poses)
        .zip(&closed)
        .map(|((g, pose), closed)| GripperDrive {
            pose: pose.as_ref(),
            closed: closed.as_ref(),
            grip: g.grip.clone(),
        })
        .collect();
    drive(
        &pass.name,
        &pass.spec,
        pass.table_top,
        &drives,
        timeline.duration,
        pass.step,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{rest_landmarks, ClothShape};

    fn sheet() -> ClothSpec {
        ClothSpec {
            shape: ClothShape::Sheet {
                size: [0.3, 0.2],
                spacing: 0.05,
            },
            workers: 1,
            ..ClothSpec::default()
        }
    }

    fn corner() -> [f64; 3] {
        rest_landmarks(&sheet(), Some(0.0)).unwrap()["corner_se"]
    }

    /// `p` raised by `lift` between `t0` and `t1`.
    fn rising(p: [f64; 3], lift: f64, t0: f64, t1: f64) -> impl Fn(f64) -> Isometry3<f64> {
        move |t: f64| {
            let u = ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
            Isometry3::translation(p[0], p[1], p[2] + lift * u)
        }
    }

    #[test]
    fn a_closed_gripper_carries_the_cloth_and_opens_on_its_signal() {
        let corner = corner();
        let pose = rising(corner, 0.06, 0.3, 0.9);
        let closed = |t: f64| (0.15..1.15).contains(&t);
        let track = drive(
            "sheet",
            &sheet(),
            Some(0.0),
            &[GripperDrive {
                pose: &pose,
                closed: &closed,
                grip: Grip {
                    radius: 0.06,
                    ..Grip::default()
                },
            }],
            2.0,
            0.1,
        )
        .unwrap();
        assert_eq!(track.name, "sheet");
        assert_eq!(track.times.first(), Some(&0.0));
        assert_eq!(track.times.last(), Some(&2.0));
        assert!(track.times.windows(2).all(|w| w[0] < w[1]));
        assert!(track.failure.is_none() && track.warnings.is_empty());
        assert_eq!(track.points.len(), track.times.len());
        assert_eq!(track.held.len(), track.times.len());
        assert_eq!(track.points[0].len(), 35);
        assert_eq!(track.triangles.len(), 48);
        let se = track.landmarks["corner_se"];
        // Open before 0.2 s, holding from there to 1.2 s, open after.
        assert!(track.held_at(0.1).is_empty());
        assert!(track.held_at(0.2).contains(&se));
        assert!(track.held_at(1.1).contains(&se));
        assert!(track.held_at(1.2).is_empty());
        // The held corner rides the gripper from where it lay when the
        // gripper closed; released, it falls back.
        let taken = track.positions_at(0.2)[se as usize];
        let up = track.positions_at(1.0)[se as usize];
        assert!(
            (up[2] - taken[2] - 0.06).abs() < 1e-5,
            "{taken:?} -> {up:?}"
        );
        assert!((taken[2] as f64 - corner[2]).abs() < 1e-3, "{taken:?}");
        let down = track.positions_at(2.0)[se as usize];
        assert!(down[2] < 0.01, "{down:?}");
        // The far corner never left the support.
        let nw = track.landmarks["corner_nw"] as usize;
        assert!(track.points.iter().all(|frame| frame[nw][2] < 0.005));
    }

    #[test]
    fn a_gripper_closing_on_a_held_patch_takes_it_over() {
        let corner = corner();
        let lifted = [corner[0], corner[1], corner[2] + 0.05];
        let giver = rising(corner, 0.05, 0.2, 0.6);
        // The taker waits where the corner arrives, closes at 0.9 s, then
        // carries it 4 cm along +x.
        let taker = move |t: f64| {
            let u = ((t - 1.0) / 0.4).clamp(0.0, 1.0);
            Isometry3::translation(lifted[0] + 0.04 * u, lifted[1], lifted[2])
        };
        let giver_closed = |t: f64| (0.15..1.55).contains(&t);
        let taker_closed = |t: f64| (0.85..1.75).contains(&t);
        let grip = Grip {
            radius: 0.06,
            ..Grip::default()
        };
        let track = drive(
            "sheet",
            &sheet(),
            Some(0.0),
            &[
                GripperDrive {
                    pose: &giver,
                    closed: &giver_closed,
                    grip: grip.clone(),
                },
                GripperDrive {
                    pose: &taker,
                    closed: &taker_closed,
                    grip,
                },
            ],
            2.0,
            0.1,
        )
        .unwrap();
        assert!(
            track.failure.is_none() && track.warnings.is_empty(),
            "{track:?}"
        );
        let se = track.landmarks["corner_se"];
        // The corner follows the taker although the giver stays closed and
        // still: it was handed over, not grasped twice.
        let handed = track.positions_at(0.9)[se as usize];
        let carried = track.positions_at(1.4)[se as usize];
        assert!(
            (carried[0] - handed[0] - 0.04).abs() < 1e-5 && (carried[2] - handed[2]).abs() < 1e-5,
            "{handed:?} -> {carried:?}"
        );
        assert!((handed[2] as f64 - lifted[2]).abs() < 1e-3, "{handed:?}");
        // The giver opening at 1.6 s drops nothing; the taker lets go at 1.8 s.
        assert!(track.held_at(1.7).contains(&se));
        assert!(track.held_at(1.8).is_empty());
        let held = track.held_at(0.5).len();
        assert_eq!(track.held_at(1.2).len(), held);
    }

    #[test]
    fn closing_on_nothing_is_a_warning_and_a_soft_release_lets_the_patch_sag() {
        let corner = corner();
        let far = |_: f64| Isometry3::translation(2.0, 0.0, 0.5);
        let pulse = |t: f64| (0.35..0.65).contains(&t);
        let pose = rising(corner, 0.05, 0.2, 0.6);
        let closed = |t: f64| (0.05..1.25).contains(&t);
        let run = |soften_steps: usize| {
            drive(
                "sheet",
                &sheet(),
                Some(0.0),
                &[
                    GripperDrive {
                        pose: &far,
                        closed: &pulse,
                        grip: Grip::default(),
                    },
                    GripperDrive {
                        pose: &pose,
                        closed: &closed,
                        grip: Grip {
                            radius: 0.06,
                            soften_steps,
                            ..Grip::default()
                        },
                    },
                ],
                1.6,
                0.1,
            )
            .unwrap()
        };
        let hard = run(0);
        assert_eq!(hard.warnings, ["gripper 0 closed on nothing at 0.40 s"]);
        let soft = run(3);
        let se = hard.landmarks["corner_se"] as usize;
        // Pinned until it opens at 1.3 s; softened over the three steps
        // before, the corner hangs below the gripper while still held.
        let pinned = hard.positions_at(1.2)[se];
        let sagging = soft.positions_at(1.2)[se];
        assert_eq!(pinned, hard.positions_at(0.9)[se]);
        assert!(
            (pinned[2] as f64 - corner[2] - 0.05).abs() < 1e-3,
            "{pinned:?}"
        );
        assert!(sagging[2] < pinned[2] - 1e-4, "{sagging:?} vs {pinned:?}");
        assert!(!soft.held_at(1.2).is_empty() && soft.held_at(1.3).is_empty());
        // Before the ramp both runs agree.
        assert_eq!(hard.positions_at(0.9), soft.positions_at(0.9));
    }

    #[test]
    fn a_rejected_step_ends_the_track_with_its_reason() {
        let corner = corner();
        // The gripper drives the patch it holds 5 cm under the support.
        let pose = move |t: f64| {
            let u = ((t - 0.5) / 0.2).clamp(0.0, 1.0);
            Isometry3::translation(corner[0], corner[1], corner[2] - 0.05 * u)
        };
        let closed = |t: f64| t > 0.05;
        let track = drive(
            "sheet",
            &sheet(),
            Some(0.0),
            &[GripperDrive {
                pose: &pose,
                closed: &closed,
                grip: Grip {
                    radius: 0.06,
                    ..Grip::default()
                },
            }],
            1.0,
            0.1,
        )
        .unwrap();
        let (reached, reason) = track.failure.clone().expect("the pass stops");
        assert!((0.45..0.7).contains(&reached), "{reached} {reason}");
        assert!(!reason.is_empty());
        assert_eq!(track.times.last(), Some(&reached));
        // Playback holds the last accepted state to the end of the cycle.
        assert_eq!(track.positions_at(1.0), track.positions_at(reached));
    }

    #[test]
    fn a_pass_checks_its_numbers() {
        let pose = |_: f64| Isometry3::identity();
        let closed = |_: f64| false;
        let gripper = |radius: f64, compliance: f64| GripperDrive {
            pose: &pose,
            closed: &closed,
            grip: Grip {
                radius,
                compliance,
                ..Grip::default()
            },
        };
        let run = |grippers: &[GripperDrive<'_>], duration: f64, step: f64| {
            drive("sheet", &sheet(), Some(0.0), grippers, duration, step)
        };
        assert!(matches!(run(&[], 1.0, 0.0), Err(ClothCellError::Pass(_))));
        assert!(matches!(
            run(&[], f64::INFINITY, 0.1),
            Err(ClothCellError::Pass(_))
        ));
        assert!(matches!(
            run(&[gripper(0.0, 0.0)], 1.0, 0.1),
            Err(ClothCellError::Pass(_))
        ));
        assert!(matches!(
            run(&[gripper(0.04, -1.0)], 1.0, 0.1),
            Err(ClothCellError::Pass(_))
        ));
        // A cycle of no length is the cloth as it lies.
        let still = run(&[], 0.0, 0.1).unwrap();
        assert_eq!(still.times, [0.0]);
        // The steps are equal and land on the end of the cycle.
        let odd = run(&[], 0.33, 0.1).unwrap();
        assert_eq!(odd.times.last(), Some(&0.33));
    }
}
