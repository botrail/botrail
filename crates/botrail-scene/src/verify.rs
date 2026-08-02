//! Post-hoc verification queries over a baked timeline — the assertion
//! layer's engine (playback, USD export, CSV and assertions all consume
//! [`SequenceTimeline`] alone). Clearance is re-scanned against the scene
//! the timeline was baked from rather than recorded during the rollout, so
//! the timeline stays a pure motion record and the sampling rate stays the
//! caller's choice.

use botrail_collide::ColliderId;

use crate::rollout::{ObjectTrack, SequenceTimeline, TrackSpan};
use crate::{Scene, SceneError};

/// The tightest robot-to-environment approach found on a timeline.
#[derive(Debug, Clone)]
pub struct Clearance {
    /// Distance in meters (0 while touching).
    pub distance: f64,
    /// When it first happens.
    pub t: f64,
    /// The touching `(robot side, obstacle)` pair when `distance == 0`,
    /// witnessed by the first sample with an actual overlap (a contact
    /// that only ever grazes exactly can leave this `None`).
    pub pair: Option<(String, String)>,
}

impl Scene {
    /// Minimum distance between the robot side (links plus carried
    /// objects) and the environment over the whole timeline, sampled every
    /// `dt` seconds. The scan replays the baked state — joints, conveyed
    /// and held objects, attachment changes — onto a clone of this scene,
    /// so `self` must be the scene the timeline was baked from (the
    /// rollout's pre-run snapshot).
    ///
    /// Robot-robot clearance is not included: inter-robot contact is
    /// already a hard rollout error (the tick check), while environment
    /// clearance is a *measure* the rollout never takes — planned motions
    /// are collision-checked, but tracking ticks and conveyed parts are
    /// not.
    ///
    /// `Ok(None)` when no sample had anything to measure (no enabled
    /// environment obstacle with collision geometry).
    pub fn timeline_min_clearance(
        &self,
        timeline: &SequenceTimeline,
        dt: f64,
    ) -> Result<Option<Clearance>, SceneError> {
        let mut world = self.clone();
        let mut span_at: Vec<Option<usize>> = vec![None; timeline.objects.len()];
        let mut best: Option<Clearance> = None;
        let samples = (timeline.duration / dt).ceil() as usize;
        for k in 0..=samples {
            let t = (k as f64 * dt).min(timeline.duration);
            apply_state(&mut world, timeline, t, &mut span_at)?;
            let Some(distance) = world.min_obstacle_distance() else {
                continue;
            };
            if best.as_ref().is_none_or(|b| distance < b.distance) {
                best = Some(Clearance {
                    distance,
                    t,
                    pair: None,
                });
            }
            if distance <= 0.0 {
                // The distance cannot get lower — the first contact's `t`
                // stands. Keep scanning for the pair: a sample that lands
                // exactly on the touch boundary has no boolean overlap yet,
                // so take the witness from the first sample that does.
                if let Some(pair) = touching_pair(&world) {
                    best.as_mut().expect("just set").pair = Some(pair);
                    break;
                }
            }
        }
        Ok(best)
    }
}

/// Replays the baked state at `t` onto `world`: every robot's joints, and
/// every tracked object's pose/attachment per its active span. `span_at`
/// carries each object's last active span index between calls (`t` must be
/// non-decreasing) so grasp/release transitions apply exactly once.
fn apply_state(
    world: &mut Scene,
    timeline: &SequenceTimeline,
    t: f64,
    span_at: &mut [Option<usize>],
) -> Result<(), SceneError> {
    for (r, track) in timeline.robots.iter().enumerate() {
        world.set_joint_positions_for(r, track.trajectory.sample(t))?;
    }
    for (i, track) in timeline.objects.iter().enumerate() {
        let Some(k) = active_span(track, t) else {
            continue;
        };
        let entered = span_at[i] != Some(k);
        span_at[i] = Some(k);
        match &track.spans[k] {
            TrackSpan::Follow {
                robot,
                link,
                offset,
                ..
            } => {
                if entered {
                    if world.attachment(&track.name).is_some() {
                        world.detach_obstacle(&track.name)?;
                    }
                    // Attach captures grasp = link_pose⁻¹ ∘ obstacle_pose,
                    // so placing the object at the span's pose first makes
                    // the captured grasp exactly `offset`; from here it
                    // rides the FK like it did during the rollout.
                    let pose = world.link_poses_for(*robot)[*link] * offset;
                    world.set_obstacle_pose(&track.name, pose)?;
                    let link_name = world.robots()[*robot].model.links[*link].name.clone();
                    world.attach_obstacle_to(*robot, &track.name, Some(&link_name), None)?;
                }
            }
            TrackSpan::Hold { pose, .. } => {
                if entered {
                    if world.attachment(&track.name).is_some() {
                        world.detach_obstacle(&track.name)?;
                    }
                    world.set_obstacle_pose(&track.name, *pose)?;
                }
            }
            TrackSpan::Linear {
                t0,
                t1,
                from,
                velocity,
            } => {
                if entered && world.attachment(&track.name).is_some() {
                    world.detach_obstacle(&track.name)?;
                }
                let mut pose = *from;
                pose.translation.vector += velocity * (t.clamp(*t0, *t1) - t0);
                world.set_obstacle_pose(&track.name, pose)?;
            }
        }
    }
    Ok(())
}

/// The active span index at `t` (same lookup as
/// [`SequenceTimeline::object_pose`]: spans tile `[0, duration]`, the last
/// span extends past its end).
fn active_span(track: &ObjectTrack, t: f64) -> Option<usize> {
    track
        .spans
        .iter()
        .position(|s| {
            let (t0, t1) = s.range();
            t >= t0 - 1e-9 && t <= t1 + 1e-9
        })
        .or(if track.spans.is_empty() {
            None
        } else {
            Some(track.spans.len() - 1)
        })
}

/// A `(robot side, environment obstacle)` collision pair at the current
/// configuration, for naming a zero-clearance contact. The robot side is a
/// link (`robot:link` with several robots) or a carried object.
fn touching_pair(world: &Scene) -> Option<(String, String)> {
    let link_name = |robot: usize, link: usize| {
        let name = world.robots()[robot].model.links[link].name.clone();
        if world.robots().len() > 1 {
            format!("{}:{}", world.robots()[robot].name, name)
        } else {
            name
        }
    };
    let robot_side = |id: &ColliderId| match id {
        ColliderId::Link { robot, link } => Some(link_name(*robot, *link)),
        ColliderId::Obstacle(k) => {
            let name = &world.obstacles()[*k].name;
            world.attachment(name).map(|_| name.clone())
        }
        ColliderId::Attached(_) => None,
    };
    let env_side = |id: &ColliderId| match id {
        ColliderId::Obstacle(k) => {
            let name = &world.obstacles()[*k].name;
            world.attachment(name).is_none().then(|| name.clone())
        }
        _ => None,
    };
    world.check_collisions().iter().find_map(|pair| {
        if let (Some(r), Some(e)) = (robot_side(&pair.a), env_side(&pair.b)) {
            Some((r, e))
        } else if let (Some(r), Some(e)) = (robot_side(&pair.b), env_side(&pair.a)) {
            Some((r, e))
        } else {
            None
        }
    })
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use botrail_model::Geometry;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    use crate::motion::{Segment, SegmentKind};
    use crate::rollout::RolloutOptions;
    use crate::seq::{Action, Condition, Device, DeviceCommand, DeviceKind, Sequence, Step};
    use crate::Scene;

    fn sample_scene() -> Scene {
        let urdf = r#"
        <robot name="r">
          <link name="a">
            <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
          </link>
          <link name="b">
            <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
          </link>
          <joint name="j" type="revolute">
            <parent link="a"/><child link="b"/>
            <origin xyz="0 0 0.5"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ))
    }

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    fn conveyor(running: bool) -> Device {
        Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.2, 0.0, 0.575),
                zone_size: Vector3::new(1.6, 0.3, 0.2),
                velocity: Vector3::new(0.25, 0.0, 0.0),
                running,
            },
        }
    }

    /// A carried sphere sweeps toward a wall sphere on the same radius —
    /// sphere-to-sphere keeps the expected clearance analytic:
    /// `2 r sin(Δθ/2) - (r₁+r₂)`, tightest at the end of the sweep.
    #[test]
    fn carried_object_clearance_is_measured_against_the_wall() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "held",
                Geometry::Sphere { radius: 0.005 },
                iso(0.1, 0.0, 0.5),
            )
            .unwrap();
        scene
            .add_obstacle(
                "wall",
                Geometry::Sphere { radius: 0.005 },
                iso(0.1 * 1.0f64.cos(), 0.1 * 1.0f64.sin(), 0.5),
            )
            .unwrap();
        scene
            .add_segment(
                "go",
                Segment {
                    kind: SegmentKind::Joint,
                    goal_positions: vec![0.8],
                    constraints: vec![],
                },
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "held".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();

        let c = scene
            .timeline_min_clearance(&tl, 0.01)
            .unwrap()
            .expect("two obstacles to measure");
        let expected = 0.2 * (0.1f64).sin() - 0.01;
        assert!(
            (c.distance - expected).abs() < 1e-6,
            "distance {} vs analytic {expected}",
            c.distance
        );
        assert!(c.pair.is_none(), "{:?}", c.pair);
        // The tightest approach is reached when the sweep completes.
        let move_end = tl.step_spans[1].end;
        assert!(
            (c.t - move_end).abs() <= 0.02,
            "t {} vs move end {move_end}",
            c.t
        );
    }

    /// A conveyed crate passes 5 mm under the arm: the rollout never
    /// checks this (no motion is planned), the clearance scan measures it.
    #[test]
    fn conveyed_crate_clearance_follows_the_linear_span() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "crate",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(-0.5, 0.0, 0.575),
            )
            .unwrap();
        scene.upsert_device(conveyor(false));
        scene.upsert_sequence(Sequence {
            name: "feed".into(),
            steps: vec![step(
                "run",
                vec![Action::Device {
                    device: "belt".into(),
                    command: DeviceCommand::Start,
                }],
                Condition::Elapsed { seconds: 4.0 },
            )],
        });
        let tl = scene
            .simulate_sequence("feed", &RolloutOptions::default())
            .unwrap();

        let c = scene
            .timeline_min_clearance(&tl, 0.01)
            .unwrap()
            .expect("crate to measure");
        // Crate bottom 0.555 vs link-b top 0.550 while their footprints
        // overlap (crate center x ∈ [-0.07, 0.07] → t ∈ [1.72, 2.28]).
        assert!((c.distance - 0.005).abs() < 1e-9, "distance {}", c.distance);
        assert!((1.7..=2.3).contains(&c.t), "t {}", c.t);
        assert!(c.pair.is_none());
    }

    /// Contact reports distance 0, the first touching time, and the pair.
    #[test]
    fn contact_reports_first_time_and_pair() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "crate",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(-0.5, 0.0, 0.55),
            )
            .unwrap();
        scene.upsert_device(conveyor(false));
        scene.upsert_sequence(Sequence {
            name: "feed".into(),
            steps: vec![step(
                "run",
                vec![Action::Device {
                    device: "belt".into(),
                    command: DeviceCommand::Start,
                }],
                Condition::Elapsed { seconds: 4.0 },
            )],
        });
        let tl = scene
            .simulate_sequence("feed", &RolloutOptions::default())
            .unwrap();

        let c = scene
            .timeline_min_clearance(&tl, 0.01)
            .unwrap()
            .expect("crate to measure");
        assert_eq!(c.distance, 0.0);
        // First overlap at crate center x = -0.07 → t = 0.43 / 0.25.
        assert!((c.t - 1.72).abs() <= 0.03, "t {}", c.t);
        assert_eq!(c.pair, Some(("b".into(), "crate".into())));
    }

    #[test]
    fn nothing_to_measure_yields_none() {
        let mut scene = sample_scene();
        scene.upsert_sequence(Sequence {
            name: "idle".into(),
            steps: vec![step("wait", vec![], Condition::Elapsed { seconds: 0.2 })],
        });
        let tl = scene
            .simulate_sequence("idle", &RolloutOptions::default())
            .unwrap();
        assert!(scene.timeline_min_clearance(&tl, 0.01).unwrap().is_none());
    }
}
