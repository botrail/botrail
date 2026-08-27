//! Timeline → USD animation baking, shared by every host: the Python
//! `SequenceTimeline.export_usd`, the wire `export_usd` request (the
//! studio's download button), and the wasm session all assemble the same
//! frames here — FK per robot per frame (vehicle bases included), object
//! tracks with stowed-visibility, weld flashes — and hand the result to
//! `botrail_usd::export`.

use botrail_scene::rollout::SequenceTimeline;
use botrail_scene::seq::{CameraMount, DeviceKind};
use botrail_scene::Scene;
use botrail_usd::export::{
    export_animation, AnimationInput, CameraSpec, CurveSpec, ExportOptions, ExportedAnimation,
    ObjectSpec, PoseTrack, RobotAnimation,
};
use nalgebra::Isometry3;

/// Bakes a window of `timeline` into an in-memory USD animation layer.
///
/// `scene` must be the pre-rollout snapshot the timeline was rolled
/// against (its FK models, base poses, and obstacle geometry are the
/// picture). `start`/`end` clip to a window of the cycle — a line's full
/// run is mostly repetition, and one steady-state takt carries the whole
/// story at a fraction of the bytes. The exported clock always starts at
/// zero, so a clipped window plays like a cycle of its own.
pub fn bake_timeline(
    scene: &Scene,
    timeline: &SequenceTimeline,
    fps: f64,
    start: Option<f64>,
    end: Option<f64>,
    asset_stem: &str,
) -> Result<ExportedAnimation, String> {
    if !(fps.is_finite() && fps > 0.0) {
        return Err(format!("fps must be positive, got {fps}"));
    }
    let from = start.unwrap_or(0.0).clamp(0.0, timeline.duration);
    let to = end
        .unwrap_or(timeline.duration)
        .clamp(0.0, timeline.duration);
    if to <= from + 1e-9 {
        return Err(format!("export window [{from}, {to}] is empty"));
    }
    let duration = to - from;
    let mut times = Vec::new();
    let mut k = 0u64;
    loop {
        let t = k as f64 / fps;
        if t >= duration - 1e-9 {
            break;
        }
        times.push(t);
        k += 1;
    }
    times.push(duration);
    // Sample times in the *timeline's* frame; `times` stays the exported
    // (zero-based) grid the exporter writes.
    let sample_at: Vec<f64> = times.iter().map(|t| t + from).collect();

    // Per-robot FK per frame (robot-major for the exporter)...
    let mut robot_frames: Vec<Vec<Vec<Isometry3<f64>>>> = Vec::with_capacity(timeline.robots.len());
    let mut joint_samples: Vec<Vec<Vec<f64>>> = Vec::with_capacity(timeline.robots.len());
    for (r, track) in timeline.robots.iter().enumerate() {
        let mut frames = Vec::with_capacity(times.len());
        let mut samples = Vec::with_capacity(times.len());
        for &t in &sample_at {
            let q = track.trajectory.sample(t);
            // A robot riding a vehicle has a base that moves, so FK has to
            // be taken against the baked base, not the parked scene.
            let poses = match SequenceTimeline::base_pose(track, t) {
                Some(base) => {
                    botrail_kin::forward_kinematics_with_base(&scene.robots()[r].model, &q, &base)
                        .map_err(|e| e.to_string())?
                }
                None => scene.fk_for(r, &q).map_err(|e| e.to_string())?,
            };
            frames.push(poses);
            samples.push(q);
        }
        robot_frames.push(frames);
        joint_samples.push(samples);
    }
    // ...and frame-major for the object tracks (handover-aware: each span
    // names its carrying robot).
    let all_frames: Vec<Vec<Vec<Isometry3<f64>>>> = (0..times.len())
        .map(|k| robot_frames.iter().map(|rf| rf[k].clone()).collect())
        .collect();

    let mut objects: Vec<ObjectSpec> = scene
        .obstacles()
        .iter()
        // A collision proxy is not part of the picture: the export is
        // what someone opens in usdview, and hidden means hidden.
        .filter(|o| o.visible)
        .map(|o| {
            let found = timeline.objects.iter().find(|t| t.name == o.name);
            // Stowed frames become animated `visibility` on the prim, so a
            // magazine of stock stays out of the picture in usdview the
            // same way it does in the studio.
            let visible: Vec<bool> = found
                .map(|track| {
                    sample_at
                        .iter()
                        .map(|&t| SequenceTimeline::object_visible(track, t))
                        .collect()
                })
                .unwrap_or_default();
            let visible = if visible.iter().all(|v| *v) {
                Vec::new()
            } else {
                visible
            };
            let track = match found {
                Some(track) => {
                    let sampled: Vec<Isometry3<f64>> = sample_at
                        .iter()
                        .enumerate()
                        .map(|(k, &t)| {
                            SequenceTimeline::object_pose(track, &all_frames[k], t)
                                .unwrap_or(o.pose)
                        })
                        .collect();
                    // A track that only blinks visibility never moves; a
                    // static xform keeps a hundred carve stages from each
                    // writing the whole frame grid.
                    match sampled.first() {
                        Some(first) if sampled.iter().all(|p| p == first) => {
                            PoseTrack::Static(*first)
                        }
                        _ => PoseTrack::Sampled(sampled),
                    }
                }
                None => PoseTrack::Static(o.pose),
            };
            ObjectSpec {
                name: o.name.clone(),
                geometry: o.geometry.clone(),
                track,
                color: o.color,
                visible,
            }
        })
        .collect();

    // Spray cones: a pale beam the cone's size, riding the TCP frame by
    // frame and visible only while the bound signal is on — the jet in
    // usdview/Omniverse follows what the studio shows.
    for flash in scene.weld_flashes() {
        if flash.kind != botrail_scene::seq::FlashKind::Spray {
            continue;
        }
        let Some(cone) = flash.cone else { continue };
        let Some(track) = timeline.signals.iter().find(|s| s.name == flash.signal) else {
            continue;
        };
        let Some(r) = scene.robot_index(&flash.robot) else {
            continue;
        };
        let tcp = scene.robots()[r].model.default_tcp_link();
        let visible: Vec<bool> = sample_at.iter().map(|&t| track.value_at(t)).collect();
        if !visible.iter().any(|v| *v) {
            continue;
        }
        // The exporter's cylinder stands along its local +Z, centred on
        // its origin; the spray runs along the TCP's -Z from the tip, so
        // the beam is offset half its length down the spray axis. A
        // cylinder rather than a cone: the geometry vocabulary the writer
        // shares with collision has no cone, and a beam reads the same
        // way at a glance.
        let offset = Isometry3::translation(0.0, 0.0, -cone.length / 2.0);
        let sampled: Vec<Isometry3<f64>> = robot_frames[r]
            .iter()
            .map(|poses| poses[tcp] * offset)
            .collect();
        objects.push(ObjectSpec {
            name: format!("effects/{}", flash.name),
            geometry: botrail_model::Geometry::Cylinder {
                radius: cone.radius * 0.6,
                length: cone.length,
            },
            track: PoseTrack::Sampled(sampled),
            color: Some([0.62, 0.78, 0.95]),
            visible,
        });
    }

    // Weld flashes: one small bright prim per current-ON interval,
    // standing where that weld happened, blinking through animated
    // visibility — so the arc shows in usdview/Omniverse exactly when the
    // baked weld-controller signal was on.
    for flash in scene.weld_flashes() {
        // Cut traces are carried by the toolpath BasisCurves; only arc
        // flashes become blinking spheres.
        if flash.kind != botrail_scene::seq::FlashKind::Flash {
            continue;
        }
        let Some(track) = timeline.signals.iter().find(|s| s.name == flash.signal) else {
            continue;
        };
        let Some(r) = scene.robot_index(&flash.robot) else {
            continue;
        };
        let robot_track = &timeline.robots[r];
        let model = &scene.robots()[r].model;
        let tcp = model.default_tcp_link();
        // Rising/falling edges -> [on, off) intervals.
        let mut intervals: Vec<(f64, f64)> = Vec::new();
        let mut on_since: Option<f64> = None;
        for &(t, value) in &track.edges {
            match (value, on_since) {
                (true, None) => on_since = Some(t),
                (false, Some(t0)) => {
                    intervals.push((t0, t));
                    on_since = None;
                }
                _ => {}
            }
        }
        if let Some(t0) = on_since {
            intervals.push((t0, timeline.duration));
        }
        // Clip to the exported window and rebase onto its clock.
        let intervals: Vec<(f64, f64)> = intervals
            .into_iter()
            .filter(|(a, b)| *b > from && *a < to)
            .map(|(a, b)| (a.max(from) - from, b.min(to) - from))
            .collect();
        for (k, &(t0, t1)) in intervals.iter().enumerate() {
            let mid = (t0 + t1) / 2.0 + from;
            let q = robot_track.trajectory.sample(mid);
            let poses = match SequenceTimeline::base_pose(robot_track, mid) {
                Some(base) => botrail_kin::forward_kinematics_with_base(model, &q, &base)
                    .map_err(|e| e.to_string())?,
                None => scene.fk_for(r, &q).map_err(|e| e.to_string())?,
            };
            let visible: Vec<bool> = times
                .iter()
                .map(|&t| t >= t0 - 1e-9 && t < t1 - 1e-9)
                .collect();
            if !visible.iter().any(|v| *v) {
                continue;
            }
            objects.push(ObjectSpec {
                name: format!("flashes/{}_{}", flash.name, k + 1),
                geometry: botrail_model::Geometry::Sphere { radius: 0.028 },
                track: PoseTrack::Static(poses[tcp]),
                color: Some([1.0, 0.82, 0.45]),
                visible,
            });
        }
    }

    // Cameras: a UsdGeomCamera per authored camera under /World/Cameras,
    // its world pose mount-resolved per frame — so "through camera" in
    // usdview frames exactly what the studio's PiP shows. botrail's -Z
    // look / +Y image-up convention is USD's, poses go over verbatim.
    let mut cameras: Vec<CameraSpec> = Vec::new();
    for camera in scene.cameras() {
        let track = match &camera.mount {
            CameraMount::World => PoseTrack::Static(camera.pose),
            CameraMount::Link { robot, link } => {
                let Some(r) = scene.robot_index(robot) else {
                    continue;
                };
                let Some(l) = scene.robots()[r].model.link_index(link) else {
                    continue;
                };
                collapse_static(
                    robot_frames[r]
                        .iter()
                        .map(|poses| poses[l] * camera.pose)
                        .collect(),
                )
            }
            CameraMount::Vehicle { device } => {
                match timeline.vehicles.iter().find(|v| &v.name == device) {
                    Some(track) => collapse_static(
                        sample_at
                            .iter()
                            .enumerate()
                            .map(|(k, &t)| {
                                SequenceTimeline::object_pose(track, &all_frames[k], t)
                                    .map(|frame| frame * camera.pose)
                                    .unwrap_or(camera.pose)
                            })
                            .collect(),
                    ),
                    // A vehicle the cycle never moved: parked at its start
                    // station. A dangling mount exports nothing.
                    None => {
                        let parked = scene.devices().iter().find(|d| &d.name == device).and_then(
                            |d| match &d.kind {
                                DeviceKind::Vehicle { path, start, .. } => path.frame_at(start),
                                _ => None,
                            },
                        );
                        match parked {
                            Some(frame) => PoseTrack::Static(frame * camera.pose),
                            None => continue,
                        }
                    }
                }
            }
        };
        // Only the aperture/focal ratio decides the framing; 20.955 "mm"
        // is the USD default horizontal aperture.
        const H_APERTURE: f64 = 20.955;
        let focal = 0.5 * H_APERTURE / (camera.fov_deg.to_radians() / 2.0).tan();
        let aspect = camera.resolution[1] as f64 / camera.resolution[0] as f64;
        cameras.push(CameraSpec {
            name: camera.name.clone(),
            track,
            focal_length: focal,
            horizontal_aperture: H_APERTURE,
            vertical_aperture: H_APERTURE * aspect,
            clipping: [camera.near, camera.far],
        });
    }

    // A sole robot keeps the historical `Robot` prim (byte compat).
    let single = timeline.robots.len() == 1;
    let names: Vec<String> = timeline
        .robots
        .iter()
        .map(|r| {
            if single {
                "Robot".to_string()
            } else {
                r.name.clone()
            }
        })
        .collect();
    let robots: Vec<RobotAnimation> = timeline
        .robots
        .iter()
        .enumerate()
        .map(|(r, _)| RobotAnimation {
            name: &names[r],
            model: &scene.robots()[r].model,
            link_poses: &robot_frames[r],
            joint_samples: Some(&joint_samples[r]),
        })
        .collect();
    let curves = toolpath_curves(scene);
    let input = AnimationInput {
        robots: &robots,
        times: &times,
        objects: &objects,
        curves: &curves,
        cameras: &cameras,
    };
    let options = ExportOptions { fps };
    export_animation(&input, &options, asset_stem).map_err(|e| e.to_string())
}

/// A sampled track whose poses never change is a static xform — the
/// object-track collapse, shared by the camera specs.
fn collapse_static(sampled: Vec<Isometry3<f64>>) -> PoseTrack {
    match sampled.first() {
        Some(first) if sampled.iter().all(|p| p == first) => PoseTrack::Static(*first),
        _ => PoseTrack::Sampled(sampled),
    }
}

/// Two `BasisCurves` overlays per toolpath — cutting (feed) polylines in
/// process orange, rapids in grey — resolved through the part frame at
/// export time. A toolpath whose frame is missing is skipped: the bake
/// itself would already have failed on it, and export stays best-effort.
fn toolpath_curves(scene: &Scene) -> Vec<CurveSpec> {
    const FEED_COLOR: [f32; 3] = [0.85, 0.33, 0.05];
    const RAPID_COLOR: [f32; 3] = [0.38, 0.38, 0.42];
    let mut specs = Vec::new();
    for tp in scene.toolpaths() {
        let Some((feed, rapid)) = botrail_scene::toolpath::overlay_polylines(scene, tp) else {
            continue;
        };
        if !feed.is_empty() {
            specs.push(CurveSpec {
                name: format!("{}_feed", tp.name),
                curves: feed,
                color: FEED_COLOR,
                width: 0.003,
            });
        }
        if !rapid.is_empty() {
            specs.push(CurveSpec {
                name: format!("{}_rapid", tp.name),
                curves: rapid,
                color: RAPID_COLOR,
                width: 0.0015,
            });
        }
    }
    specs
}
