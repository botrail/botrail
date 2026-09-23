//! Timeline → USD animation baking, shared by every host: the Python
//! `SequenceTimeline.export_usd`, the wire `export_usd` request (the
//! studio's download button), and the wasm session all assemble the same
//! frames here — FK per robot per frame (vehicle bases included), object
//! tracks with stowed-visibility, weld flashes — and hand the result to
//! `botrail_usd::export`.

use botrail_scene::physics_world::PlanKind;
use botrail_scene::rollout::{PhysicsOptions, PhysicsScope, SequenceTimeline};
use botrail_scene::seq::{CameraMount, DeviceKind};
use botrail_scene::Scene;
use botrail_usd::export::{
    export_animation, export_simulation, AnimationInput, ArticulationSpec, BeltSpec, BodyRole,
    CameraSpec, CarrierSpec, CurveSpec, ExportOptions, ExportedAnimation, ObjectBody, ObjectSpec,
    PhysicsSpec, PoseTrack, Ride, RobotAnimation, ServoSpec, SimulationSpec, UnitSpec,
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
/// The UsdPhysics spec for an obstacle, from its resolved body
/// properties (part-identity mass included) — `None` for un-annotated
/// scenery, which exports exactly as before.
fn physics_spec(scene: &Scene, name: &str) -> Option<botrail_usd::export::PhysicsSpec> {
    let props = scene.resolved_body_props(name)?;
    Some(botrail_usd::export::PhysicsSpec {
        dynamic: props.kind == botrail_physics::BodyKind::Dynamic,
        mass: (props.kind == botrail_physics::BodyKind::Dynamic)
            .then_some(props.mass)
            .flatten(),
        friction: props.material.friction,
        restitution: props.material.restitution,
    })
}

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
                material: o.material.map(|m| botrail_usd::export::SurfaceMaterial {
                    metalness: m.metalness,
                    roughness: m.roughness,
                    opacity: m.opacity,
                }),
                visual_asset: o.visual_asset.clone(),
                name: o.name.clone(),
                geometry: o.geometry.clone(),
                track,
                color: o.color,
                visible,
                physics: physics_spec(scene, &o.name),
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
            material: None,
            visual_asset: None,
            name: format!("effects/{}", flash.name),
            geometry: botrail_model::Geometry::Cylinder {
                radius: cone.radius * 0.6,
                length: cone.length,
            },
            track: PoseTrack::Sampled(sampled),
            color: Some([0.62, 0.78, 0.95]),
            visible,
            physics: None,
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
                material: None,
                visual_asset: None,
                name: format!("flashes/{}_{}", flash.name, k + 1),
                geometry: botrail_model::Geometry::Sphere { radius: 0.028 },
                track: PoseTrack::Static(poses[tcp]),
                color: Some([1.0, 0.82, 0.45]),
                visible,
                physics: None,
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
            resolution: camera.resolution,
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

/// Bakes the scene as it stands — no timeline — into a static USD layer:
/// every robot at its current joint positions, every visible obstacle at
/// its pose, toolpath overlays, cameras at their mount-resolved frames
/// (a vehicle-mounted camera at the parked frame). The cell a layout is
/// handed around as, rather than a cycle of it.
pub fn bake_scene(scene: &Scene, asset_stem: &str) -> Result<ExportedAnimation, String> {
    bake_scene_stage(scene, None, asset_stem)
}

/// Bakes the scene as it stands into a *simulation* stage
/// (design-world-physics.md W4-U): the world `physics_plan(options)`
/// tabulates, written as UsdPhysics for an engine to own — Isaac Sim /
/// Isaac Lab first. Robots are articulations, dynamic units rigid bodies,
/// what a device moves kinematic bodies, the rest static colliders; a
/// conveyor's belt carries a surface velocity. No timeSamples: an
/// animation and a simulation would fight over the same prims.
pub fn bake_simulation(
    scene: &Scene,
    options: &PhysicsOptions,
    asset_stem: &str,
) -> Result<ExportedAnimation, String> {
    bake_scene_stage(scene, Some(options), asset_stem)
}

/// How far below its authored box a conveyor zone still claims the body
/// under it (m) — the belt's top face *is* the zone's bottom face, so the
/// two only touch. The physics bake's own `ZONE_MARGIN`.
const BELT_MARGIN: f64 = 0.005;

/// The simulation-stage view of the obstacles: which to export, which
/// rigid body each is part of, and those bodies.
struct SimulationBodies {
    /// Obstacle indices exported, in obstacle order.
    exported: Vec<usize>,
    bodies: Vec<ObjectBody>,
    colliders: Vec<Option<PhysicsSpec>>,
    units: Vec<UnitSpec>,
    /// The one body of each rigid device that has one, by device name —
    /// what a robot riding the device is bolted to.
    device_units: std::collections::HashMap<String, usize>,
}

/// Mass (kg) of a vehicle body that is a link of its rider's articulation
/// but has nothing to weigh — every piece of it a picture. A link needs a
/// mass; the virtual joints that carry it do not care which.
const BARE_CARRIER_MASS: f64 = 1.0;

/// `ridden` names the vehicles that carry a robot the stage authors as an
/// articulation: their body is a link of that articulation, not a
/// kinematic body of its own.
fn simulation_bodies(
    scene: &Scene,
    options: &PhysicsOptions,
    ridden: &std::collections::HashSet<&str>,
) -> SimulationBodies {
    let obstacles = scene.obstacles();
    let world = options.scope == PhysicsScope::World;
    let default_material = botrail_physics::BodyProps::default().material;

    // What a device moves by name is kinematic: the consumer puts it where
    // its own controller says. A vehicle's body, an axis's objects and a
    // lift's car each move as one piece, so each is *one* body (posing an
    // AMR is one call, not one per cover and wheel); a source's pool is
    // so many separate parts.
    let index_of: std::collections::HashMap<&str, usize> = obstacles
        .iter()
        .enumerate()
        .map(|(i, o)| (o.name.as_str(), i))
        .collect();
    // A floating machine's vehicle is a planning device: its body (the
    // massing footprint drawn around a walker, to the head) is not matter
    // the machine can stand in, so the bake keeps it out of the world —
    // and so does the stage. Drawn, if it is drawn; never a collider.
    let phantom: std::collections::HashSet<&str> = scene
        .robots()
        .iter()
        .enumerate()
        .filter(|(r, _)| scene.robot_physics(*r, options).1)
        .filter_map(|(_, robot)| robot.mount.as_ref().map(|m| m.device.as_str()))
        .collect();
    let mut ghosts: std::collections::HashSet<usize> = Default::default();
    // (what to call the body, its listed obstacles, the device's name when
    // the body is that device's one body)
    let mut driven: Vec<(String, Vec<usize>, Option<String>)> = Vec::new();
    for device in scene.devices() {
        let (names, rigid) = match &device.kind {
            DeviceKind::LinearAxis { objects, .. } => (objects, true),
            DeviceKind::Vehicle { body, .. } => (body, true),
            DeviceKind::Lift { car, .. } => (car, true),
            DeviceKind::Source { pool, .. } => (pool, false),
            DeviceKind::Conveyor { .. } | DeviceKind::Sink { .. } => continue,
        };
        let listed: Vec<usize> = names
            .iter()
            .filter_map(|n| index_of.get(n.as_str()).copied())
            .collect();
        if phantom.contains(device.name.as_str()) {
            ghosts.extend(listed);
            continue;
        }
        if rigid {
            driven.push((device.name.clone(), listed, Some(device.name.clone())));
        } else {
            driven.extend(
                listed
                    .into_iter()
                    .map(|i| (obstacles[i].name.clone(), vec![i], None)),
            );
        }
    }
    // The belt under a conveyor zone: a fixed body whose collision
    // reaches into the zone — the same "contacts inside the box" the
    // bake's surface-velocity zone drives.
    let belts: std::collections::HashMap<usize, BeltSpec> = scene
        .belt_obstacles(BELT_MARGIN)
        .into_iter()
        .filter_map(|(i, d)| match &scene.devices()[d].kind {
            DeviceKind::Conveyor {
                velocity, running, ..
            } => Some((
                i,
                BeltSpec {
                    velocity: [velocity.x, velocity.y, velocity.z],
                    running: *running,
                },
            )),
            _ => None,
        })
        .collect();

    let mut units: Vec<UnitSpec> = Vec::new();
    let mut device_units: std::collections::HashMap<String, usize> = Default::default();
    let mut unit_of: Vec<Option<usize>> = vec![None; obstacles.len()];
    let mut material_of = vec![default_material; obstacles.len()];
    // What rides a fixed unit's root (the world scope folds a door's
    // handle into the door): it goes wherever the root goes.
    let mut riders: std::collections::HashMap<usize, Vec<usize>> = Default::default();
    for unit in scene.physics_units(options) {
        if unit.kind != PlanKind::Dynamic {
            riders.insert(unit.frame, unit.members);
            continue;
        }
        for &i in &unit.members {
            material_of[i] = unit.props.material;
            unit_of[i] = Some(units.len());
        }
        units.push(UnitSpec {
            name: unit.name.clone(),
            role: BodyRole::Dynamic,
            pose: obstacles[unit.frame].pose,
            mass: unit.props.mass,
            belt: None,
        });
    }
    for (device, listed, rigid_device) in driven {
        let mut members: Vec<usize> = Vec::new();
        for i in listed {
            for &m in riders.get(&i).unwrap_or(&vec![i]) {
                if unit_of[m].is_none() && !members.contains(&m) {
                    members.push(m);
                }
            }
        }
        let Some(&first) = members.first() else {
            continue;
        };
        let name = driven_body_name(&device, &members, obstacles);
        let frame = index_of.get(name.as_str()).copied().unwrap_or(first);
        for &m in &members {
            unit_of[m] = Some(units.len());
        }
        // A vehicle that carries a robot is that robot's base: a link the
        // solver moves through the robot's virtual base joints, not a body
        // posed from outside.
        let link = rigid_device.as_deref().is_some_and(|d| ridden.contains(d));
        let weighs = members
            .iter()
            .any(|&m| obstacles[m].enabled || (world && obstacles[m].walkable));
        if let Some(rigid_device) = rigid_device {
            device_units.insert(rigid_device, units.len());
        }
        units.push(UnitSpec {
            name,
            role: if link {
                BodyRole::Link
            } else {
                BodyRole::Kinematic
            },
            pose: obstacles[frame].pose,
            mass: (link && !weighs).then_some(BARE_CARRIER_MASS),
            // A roller deck on a vehicle: the body carries its belt.
            belt: members.iter().find_map(|m| belts.get(m).copied()),
        });
    }
    for (i, o) in obstacles.iter().enumerate() {
        if unit_of[i].is_some() {
            continue;
        }
        if let Some(props) = &o.physics {
            material_of[i] = props.material;
        }
        // Fixed scenery is lowered obstacle by obstacle; a belt still
        // needs a body to hang its velocity on.
        if let Some(belt) = belts.get(&i).copied() {
            unit_of[i] = Some(units.len());
            units.push(UnitSpec {
                name: o.name.clone(),
                role: BodyRole::Kinematic,
                pose: o.pose,
                mass: None,
                belt: Some(belt),
            });
        }
    }

    let mut out = SimulationBodies {
        exported: Vec::new(),
        bodies: Vec::new(),
        colliders: Vec::new(),
        units,
        device_units,
    };
    for (i, o) in obstacles.iter().enumerate() {
        // The engine's truth is `enabled` (a floor slab a walker stands on
        // is a floor to the world scope too); the picture's is `visible`.
        let collides = (o.enabled || (world && o.walkable)) && !ghosts.contains(&i);
        if !collides && !o.visible {
            continue;
        }
        out.exported.push(i);
        out.bodies.push(ObjectBody {
            unit: unit_of[i],
            guide: !o.visible,
        });
        out.colliders.push(collides.then_some(PhysicsSpec {
            dynamic: false,
            mass: None,
            friction: material_of[i].friction,
            restitution: material_of[i].restitution,
        }));
    }
    out
}

/// What to call the one body of a device's obstacles: the `/`-prefix
/// their names share (`amr1` for `amr1/base_link` and `amr1/visual/...`, a
/// lone obstacle's own name), so the body sits where its pieces already
/// are — unless something that is *not* part of the device lives under
/// that prefix too (two doors of one `cell/`), which would then be moved
/// out from under the body; the device's own name serves there.
fn driven_body_name(
    device: &str,
    members: &[usize],
    obstacles: &[botrail_scene::Obstacle],
) -> String {
    let mut shared: Vec<&str> = obstacles[members[0]].name.split('/').collect();
    for &m in &members[1..] {
        let segments: Vec<&str> = obstacles[m].name.split('/').collect();
        let agree = shared
            .iter()
            .zip(&segments)
            .take_while(|(a, b)| a == b)
            .count();
        shared.truncate(agree);
    }
    let shared = shared.join("/");
    let free = |name: &str| {
        !name.is_empty()
            && obstacles.iter().enumerate().all(|(i, o)| {
                members.contains(&i) || !(o.name == name || o.name.starts_with(&format!("{name}/")))
            })
    };
    if free(&shared) {
        shared
    } else if free(device) {
        device.to_string()
    } else {
        format!("{device}_body")
    }
}

fn articulation_spec(scene: &Scene, robot: usize, options: &PhysicsOptions) -> ArticulationSpec {
    let (dynamics, floating) = scene.robot_physics(robot, options);
    ArticulationSpec {
        floating,
        powered: options.powered != Some(false),
        passive_damping: options.passive_damping,
        servos: dynamics
            .map(|d| {
                d.servos
                    .iter()
                    .map(|servo| ServoSpec {
                        max_force: Some(servo.max_force),
                        max_velocity: Some(servo.max_velocity),
                        armature: Some(servo.armature),
                    })
                    .collect()
            })
            .unwrap_or_default(),
    }
}

fn bake_scene_stage(
    scene: &Scene,
    physics: Option<&PhysicsOptions>,
    asset_stem: &str,
) -> Result<ExportedAnimation, String> {
    let times = [0.0];

    let mut robot_frames: Vec<Vec<Vec<Isometry3<f64>>>> = Vec::with_capacity(scene.robots().len());
    let mut joint_samples: Vec<Vec<Vec<f64>>> = Vec::with_capacity(scene.robots().len());
    for (r, robot) in scene.robots().iter().enumerate() {
        let q = robot.joint_positions().to_vec();
        let poses = scene.fk_for(r, &q).map_err(|e| e.to_string())?;
        robot_frames.push(vec![poses]);
        joint_samples.push(vec![q]);
    }

    // Which vehicle each robot rides, when its base is not free. The first
    // robot on a vehicle takes it as its base; the next ones join that
    // articulation. A USD-sourced robot rides like any other — unless its
    // stage roots the articulation on the base body itself, which the
    // exporter cannot re-root (the model names such a root link by the
    // root prim's own path).
    let mut riders: Vec<Option<&str>> = vec![None; scene.robots().len()];
    let mut hosts: std::collections::HashMap<&str, usize> = Default::default();
    let mut rides: Vec<Option<Ride>> = vec![None; scene.robots().len()];
    let mut stranded: Vec<String> = Vec::new();
    if let Some(options) = physics {
        for (r, robot) in scene.robots().iter().enumerate() {
            let Some(mount) = robot.mount.as_ref() else {
                continue;
            };
            if scene.robot_physics(r, options).1 {
                continue; // a floating base rides nothing
            }
            let model = &robot.model;
            if model
                .source
                .usd_stage()
                .is_some_and(|(_, root)| model.links[model.root_link].name == root)
            {
                stranded.push(format!(
                    "{} (on {}): its stage roots the articulation on the base body itself; the \
                     robot stays anchored to the world as its stage says and does not ride",
                    robot.name, mount.device
                ));
                continue;
            }
            riders[r] = Some(mount.device.as_str());
            if let Some(&host) = hosts.get(mount.device.as_str()) {
                rides[r] = Some(Ride::Joins { host });
            } else {
                hosts.insert(mount.device.as_str(), r);
            }
        }
    }
    let ridden: std::collections::HashSet<&str> = hosts.keys().copied().collect();
    let simulation = physics.map(|options| simulation_bodies(scene, options, &ridden));
    let exported: Vec<usize> = match &simulation {
        Some(sim) => sim.exported.clone(),
        // A collision proxy is not part of the picture: the export is
        // what someone opens in usdview, and hidden means hidden.
        None => (0..scene.obstacles().len())
            .filter(|&i| scene.obstacles()[i].visible)
            .collect(),
    };
    let objects: Vec<ObjectSpec> = exported
        .iter()
        .enumerate()
        .map(|(k, &i)| {
            let o = &scene.obstacles()[i];
            ObjectSpec {
                material: o.material.map(|m| botrail_usd::export::SurfaceMaterial {
                    metalness: m.metalness,
                    roughness: m.roughness,
                    opacity: m.opacity,
                }),
                visual_asset: o.visual_asset.clone(),
                name: o.name.clone(),
                geometry: o.geometry.clone(),
                track: PoseTrack::Static(o.pose),
                color: o.color,
                visible: Vec::new(),
                physics: match &simulation {
                    Some(sim) => sim.colliders[k],
                    None => physics_spec(scene, &o.name),
                },
            }
        })
        .collect();

    let mut cameras: Vec<CameraSpec> = Vec::new();
    for camera in scene.cameras() {
        let pose = match &camera.mount {
            CameraMount::World => camera.pose,
            CameraMount::Link { robot, link } => {
                let Some(r) = scene.robot_index(robot) else {
                    continue;
                };
                let Some(l) = scene.robots()[r].model.link_index(link) else {
                    continue;
                };
                robot_frames[r][0][l] * camera.pose
            }
            CameraMount::Vehicle { device } => {
                // No cycle ran: the vehicle stands at its start station.
                let parked = scene
                    .devices()
                    .iter()
                    .find(|d| &d.name == device)
                    .and_then(|d| match &d.kind {
                        DeviceKind::Vehicle { path, start, .. } => path.frame_at(start),
                        _ => None,
                    });
                match parked {
                    Some(frame) => frame * camera.pose,
                    None => continue,
                }
            }
        };
        const H_APERTURE: f64 = 20.955;
        let focal = 0.5 * H_APERTURE / (camera.fov_deg.to_radians() / 2.0).tan();
        let aspect = camera.resolution[1] as f64 / camera.resolution[0] as f64;
        cameras.push(CameraSpec {
            name: camera.name.clone(),
            track: PoseTrack::Static(pose),
            focal_length: focal,
            horizontal_aperture: H_APERTURE,
            vertical_aperture: H_APERTURE * aspect,
            clipping: [camera.near, camera.far],
            resolution: camera.resolution,
        });
    }

    // A sole robot keeps the historical `Robot` prim (byte compat).
    let single = scene.robots().len() == 1;
    let names: Vec<String> = scene
        .robots()
        .iter()
        .map(|r| {
            if single {
                "Robot".to_string()
            } else {
                r.name.clone()
            }
        })
        .collect();
    let robots: Vec<RobotAnimation> = scene
        .robots()
        .iter()
        .enumerate()
        .map(|(r, robot)| RobotAnimation {
            name: &names[r],
            model: &robot.model,
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
    let (Some(sim), Some(physics)) = (&simulation, physics) else {
        let options = ExportOptions { fps: 60.0 };
        return export_animation(&input, &options, asset_stem).map_err(|e| e.to_string());
    };
    // A USD-sourced robot references its own stage, physics included; the
    // rest are authored as articulations here.
    // A robot riding a vehicle takes the vehicle in as its base: six virtual
    // joints state the vehicle's pose, and driving them moves both. The
    // vehicle's frame is where the mount says it is — the robot's base,
    // less the mount's offset — so it is right wherever the scene stands.
    for (r, robot) in scene.robots().iter().enumerate() {
        if rides[r].is_some() {
            continue; // joins a host
        }
        if let Some((device, mount)) = riders[r].zip(robot.mount.as_ref()) {
            rides[r] = Some(Ride::Base(CarrierSpec {
                unit: sim.device_units.get(device).copied(),
                frame: robot.base_pose() * mount.offset.inverse(),
            }));
        }
    }
    let articulations: Vec<Option<ArticulationSpec>> = scene
        .robots()
        .iter()
        .enumerate()
        .map(|(r, robot)| {
            robot
                .model
                .source
                .usd_stage()
                .is_none()
                .then(|| articulation_spec(scene, r, physics))
        })
        .collect();
    let spec = SimulationSpec {
        units: &sim.units,
        bodies: &sim.bodies,
        articulations: &articulations,
        rides: &rides,
        ground: physics.ground,
    };
    let mut exported = export_simulation(&input, &spec, asset_stem).map_err(|e| e.to_string())?;
    exported.warnings.extend(stranded);
    let belts: Vec<&str> = sim
        .units
        .iter()
        .filter(|u| u.belt.is_some())
        .map(|u| u.name.as_str())
        .collect();
    if !belts.is_empty() {
        // Not a defect of the stage, but the one thing about it that fails
        // silently on the consumer's side (measured on Isaac Sim 5.1).
        exported.warnings.push(format!(
            "conveyor belt bodies ({}) carry PhysxSurfaceVelocityAPI, which PhysX honours under \
             CPU dynamics only: under GPU dynamics parts fall through a running belt, and the \
             physics replicator does not clone it (Isaac Lab: device=\"cpu\" and \
             replicate_physics=False, or switch physxSurfaceVelocity:surfaceVelocityEnabled off)",
            belts.join(", ")
        ));
    }
    Ok(exported)
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
