//! Baked-recording import: read a stage whose prims carry transform (and
//! optionally `JointStateAPI`) timeSamples — an Isaac Sim recording or a
//! botrail export — and lift it back into botrail-world tracks.
//!
//! Two tiers, mirroring the writer:
//! 1. **Joint state** — when every actuated joint carries a readable
//!    `state:{angular,linear}:physics:position`, q(t) is read directly
//!    (degrees / stage units converted) and cross-checked against the
//!    recorded body transforms by FK residual.
//! 2. **Transforms** — otherwise the body prims' composed world transforms
//!    become per-link pose tracks, the constraint-agnostic view of the
//!    recording (clients replay them with `setLinkTransforms`).
//!
//! The writer also has two shapes of robot, and the reader follows: a
//! USD-sourced robot is a `references` arc plus `over`s, addressed by prim
//! path (either tier applies); a *baked* robot (URDF or composite source)
//! is one flat `def Xform` per sanitized link name with world poses and no
//! joint prims — those robots resolve by that naming and always play as
//! transforms.
//!
//! Sampling walks the recording's own integer time codes (recorders author
//! per frame), so no interpolation semantics leak in. Each robot may live
//! at any prim path — `/World/<instance>` in a botrail export, the robot
//! stage's own root when animation is layered directly, or wherever a
//! simulator parked it; roots come from explicit options, the export
//! convention, or (sole robot) structural search.

use std::path::{Path, PathBuf};

use botrail_kin::forward_kinematics_with_base;
use botrail_model::{JointType, RobotModel};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use openusd::schemas::geom::Xformable;
use openusd::usd::{Prim, SchemaBase, Stage, TimeCode};
use openusd::{gf, sdf};

use crate::export::{
    baked_robot_stage_info_on, find_baked_robot_root, find_robot_root, open_stage_prims,
    remap_model_path, robot_stage_info_on, sanitize_name, stage_frame_metadata, OpenedStage,
    RobotStageInfo, UsdExportError,
};
use crate::{decompose_matrix, AnyPrim, UsdImportError};

#[derive(Debug, Clone, Default)]
pub struct RecordingImportOptions {
    /// Extra resolver roots (worth pointing at the robot's own stage
    /// directory when the recording references it relatively).
    pub search_paths: Vec<PathBuf>,
    /// Ignore `JointStateAPI` even when present and use the transform tier.
    pub force_transforms: bool,
    /// Explicit robot roots: `(instance name, prim path in the recording)`.
    /// Robots not listed fall back to the export convention
    /// (`/World/<sanitized name>`); a sole robot additionally falls back to
    /// structural search (which cannot disambiguate identical twins).
    pub robot_roots: Vec<(String, String)>,
}

/// How the robot part of the recording is best played back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordingMode {
    /// q(t) recovered from `JointStateAPI` — joint-space playback.
    JointState,
    /// Body transforms only — link-pose (baked) playback.
    Transforms,
}

/// One robot's share of an imported recording.
#[derive(Debug)]
pub struct ImportedRobotRecording {
    /// Instance name (as passed to [`import_recording`]).
    pub name: String,
    pub mode: RecordingMode,
    /// q per frame (DOF order); present in [`RecordingMode::JointState`].
    pub joint_samples: Option<Vec<Vec<f64>>>,
    /// World link poses per frame (botrail Z-up meters), aligned with
    /// `model.links` — always present (playback and validation).
    pub link_poses: Vec<Vec<Isometry3<f64>>>,
}

#[derive(Debug)]
pub struct ImportedRecording {
    /// Frame times in seconds from 0 (the recording's own frame grid).
    pub times: Vec<f64>,
    /// One entry per robot, in input order.
    pub robots: Vec<ImportedRobotRecording>,
    /// Obstacles that move during the recording: `(name, world pose per
    /// frame)` in the scene importer's obstacle-pose convention.
    pub object_tracks: Vec<(String, Vec<Isometry3<f64>>)>,
    pub warnings: Vec<String>,
}

/// Per-robot structural facts resolved against the recording.
struct RobotResolution {
    name: String,
    info: RobotStageInfo,
    robot_up_fix: UnitQuaternion<f64>,
    /// Range of this robot's links inside the tracked-path table.
    start: usize,
    /// `(stage_root, model_root)` when the robot replays against its
    /// source stage — prim-path naming, `JointStateAPI` eligible. Baked
    /// robots (URDF or composite sources) carry `None`: flat sanitized
    /// link names, transform playback only.
    referenced: Option<(String, String)>,
}

/// Reads a recording composed at `path` against the given robot models —
/// `(instance name, model)` in scene order. `obstacle_names` are the
/// scene's obstacles; each is looked up by prim path (or under the export
/// convention `/World/Env/<name>`) and gets a motion track when it
/// actually moves.
///
/// Robot roots resolve, per robot: an explicit entry in
/// [`RecordingImportOptions::robot_roots`], else the export convention
/// `/World/<sanitized name>`, else (sole robot only) a structural search.
pub fn import_recording(
    path: &Path,
    robots: &[(String, &RobotModel)],
    obstacle_names: &[String],
    options: &RecordingImportOptions,
) -> Result<ImportedRecording, UsdImportError> {
    if robots.is_empty() {
        return Err(UsdImportError::Recording("no robots".into()));
    }
    let recording = |e: crate::export::UsdExportError| match e {
        // The shared opener speaks of "robot stages"; here it opened the
        // recording itself.
        crate::export::UsdExportError::Open { path, message } => {
            UsdImportError::Recording(format!("failed to open `{path}`: {message}"))
        }
        other => UsdImportError::Recording(other.to_string()),
    };

    let mut warnings = Vec::new();
    let opened = open_stage_prims(path, &options.search_paths).map_err(recording)?;

    let mut resolutions: Vec<RobotResolution> = Vec::with_capacity(robots.len());
    let mut tracked_paths: Vec<String> = Vec::new();
    for (name, model) in robots {
        let sole = robots.len() == 1;
        let (info, robot_up_fix, referenced) = match model.source.usd_stage() {
            Some((robot_stage_path, articulation_root)) => {
                let robot_stage_path = robot_stage_path.to_path_buf();
                let model_root = articulation_root.to_string();
                let stage_root = resolve_stage_root(&opened, &options, name, sole, || {
                    find_robot_root(&opened, &model_root, model)
                })?;
                // The recording composes the robot subtree through whatever
                // corrective transform it likes, so a scaled robot
                // legitimately shows "non-rigid" here — the chain scale is
                // handled below; drop that noise.
                let mut info_warnings = Vec::new();
                let info = robot_stage_info_on(
                    &opened,
                    &stage_root,
                    &model_root,
                    model,
                    &mut info_warnings,
                )
                .map_err(recording)?;
                warnings.extend(
                    info_warnings
                        .into_iter()
                        .filter(|w| !w.contains("non-rigid")),
                );

                // Botrail link axes were fixed at robot-import time by the
                // *robot stage's* up axis: raw body axes relabel to botrail
                // by its `F`, no matter how the recording embeds the
                // subtree. Peel it off on the right.
                let robot_up_fix =
                    match stage_frame_metadata(&robot_stage_path, &options.search_paths) {
                        Ok(f) => f.up_fix,
                        Err(e) => {
                            warnings.push(format!(
                                "robot stage unreadable ({e}); assuming its axes match the \
                                 recording's"
                            ));
                            opened.frame.up_fix
                        }
                    };
                (info, robot_up_fix, Some((stage_root, model_root)))
            }
            None => {
                // Baked robots (URDF or composite sources) have no stage to
                // replay against: their exporter (`author_urdf_robot`)
                // wrote one flat prim per link with world-pose samples in
                // botrail axes, no joint prims, no `JointStateAPI`. Walk
                // the same naming back; `K` and the robot-stage axis
                // factor are both identity, and only the transform tier
                // can apply.
                let stage_root = resolve_stage_root(&opened, &options, name, sole, || {
                    find_baked_robot_root(&opened, model)
                })?;
                let info =
                    baked_robot_stage_info_on(&opened, &stage_root, model).map_err(recording)?;
                (info, UnitQuaternion::identity(), None)
            }
        };
        let start = tracked_paths.len();
        tracked_paths.extend(info.links.iter().map(|l| l.stage_path.clone()));
        resolutions.push(RobotResolution {
            name: name.clone(),
            info,
            robot_up_fix,
            start,
            referenced,
        });
    }

    // ---- obstacle prim resolution ---------------------------------------
    // Two homes per obstacle: its own prim path (an Isaac recording over
    // the original stage) or the export convention `/World/Env/...` with
    // `/`-segmented names nested and each segment sanitized.
    let mut object_paths: Vec<(String, String)> = Vec::new();
    for name in obstacle_names {
        let candidates = [name.clone(), env_path_of(name)];
        match candidates.iter().find(|p| opened.has_prim(p)) {
            Some(p) => object_paths.push((name.clone(), p.clone())),
            None if name.starts_with('/') => warnings.push(format!(
                "obstacle `{name}` has no prim in the recording; left static"
            )),
            None => {}
        }
    }

    // ---- time range: layer metadata, else scanned sample codes ----------
    let stage = &opened.stage;
    let link_count = tracked_paths.len();
    tracked_paths.extend(object_paths.iter().map(|(_, p)| p.clone()));

    let (mut start, mut end, mut tcps) = (None, None, 24.0f64);
    {
        let layer = stage.root_layer();
        if let Some(root) = layer.pseudo_root() {
            if let Ok(Some(v)) = root.field("startTimeCode") {
                start = value_to_f64(&v);
            }
            if let Ok(Some(v)) = root.field("endTimeCode") {
                end = value_to_f64(&v);
            }
            if let Ok(Some(v)) = root.field("timeCodesPerSecond") {
                if let Some(t) = value_to_f64(&v).filter(|t| *t > 0.0) {
                    tcps = t;
                }
            }
        }
    }
    if start.is_none() || end.is_none() {
        let (lo, hi) = scan_sample_range(stage, &tracked_paths);
        start = start.or(lo);
        end = end.or(hi);
    }
    let (Some(start), Some(end)) = (start, end) else {
        return Err(UsdImportError::Recording(
            "no time range: the stage has neither start/endTimeCode metadata nor timeSamples on \
             the robot or obstacle prims"
                .into(),
        ));
    };
    if end < start {
        return Err(UsdImportError::Recording(format!(
            "bad time range [{start}, {end}]"
        )));
    }
    // The recording's own frame grid: integer codes hit authored samples,
    // so interpolation semantics never matter.
    let codes: Vec<f64> = (start.floor() as i64..=end.ceil() as i64)
        .map(|c| c as f64)
        .collect();
    let times: Vec<f64> = codes.iter().map(|c| (c - codes[0]) / tcps).collect();

    // ---- per-frame world transforms of tracked prims --------------------
    let chains = build_chains(stage, &tracked_paths)?;
    let worlds: Vec<Vec<(Isometry3<f64>, f64)>> = codes
        .iter()
        .map(|&code| evaluate_chains(&chains, code))
        .collect();

    // Recording frame conventions apply to every track equally.
    let rec_frame = opened.frame;

    let mut out_robots = Vec::with_capacity(robots.len());
    for ((_, model), res) in robots.iter().zip(&resolutions) {
        let info = &res.info;
        let n_links = info.links.len();
        // Robot links: body → link frame (`∘ K`), then stage coords →
        // botrail: translation and the rotation's left side follow the
        // *recording's* frame; the rotation's right side unlabels the
        // *robot stage's* axes (the two coincide only when animation is
        // layered straight onto the robot stage). `K` is authored in the
        // robot stage's units; the chain's accumulated scale re-expresses
        // it in recording units (a cm robot composed into a meter
        // recording).
        let ru_inv = res.robot_up_fix.inverse();
        let link_poses: Vec<Vec<Isometry3<f64>>> = worlds
            .iter()
            .map(|frame| {
                (0..n_links)
                    .map(|i| {
                        let (body, scale) = frame[res.start + i];
                        let k = info.links[i].k_inv.inverse();
                        let k_scaled = Isometry3::from_parts(
                            Translation3::from(k.translation.vector * scale),
                            k.rotation,
                        );
                        let raw = body * k_scaled;
                        Isometry3::from_parts(
                            Translation3::from(
                                rec_frame.up_fix * (raw.translation.vector * rec_frame.mpu),
                            ),
                            rec_frame.up_fix * raw.rotation * ru_inv,
                        )
                    })
                    .collect()
            })
            .collect();

        // ---- tier 1: JointStateAPI --------------------------------------
        // Only referenced robots can carry it; a baked export never
        // authors joint prims, so those go straight to the transform tier.
        let joint_samples = match (&res.referenced, options.force_transforms) {
            (Some((stage_root, model_root)), false) => read_joint_states(
                stage,
                model,
                info,
                stage_root,
                model_root,
                &codes,
                &mut warnings,
            ),
            _ => None,
        };
        let mode = if joint_samples.is_some() {
            RecordingMode::JointState
        } else {
            RecordingMode::Transforms
        };
        if let Some(samples) = &joint_samples {
            check_fk_residual(model, &link_poses, samples, &mut warnings);
        }
        out_robots.push(ImportedRobotRecording {
            name: res.name.clone(),
            mode,
            joint_samples,
            link_poses,
        });
    }

    // Obstacles: geometry convention (vertices were baked at import), and
    // only the ones that actually move.
    let mut object_tracks = Vec::new();
    for (offset, (name, _)) in object_paths.iter().enumerate() {
        let idx = link_count + offset;
        let track: Vec<Isometry3<f64>> = worlds
            .iter()
            .map(|frame| {
                let raw = &frame[idx].0;
                Isometry3::from_parts(
                    Translation3::from(rec_frame.up_fix * (raw.translation.vector * rec_frame.mpu)),
                    rec_frame.up_fix * raw.rotation,
                )
            })
            .collect();
        let rest = track[0];
        let moved = track.iter().any(|p| {
            (p.translation.vector - rest.translation.vector).norm() > 1e-9
                || p.rotation.angle_to(&rest.rotation) > 1e-9
        });
        if moved {
            object_tracks.push((name.clone(), track));
        }
    }

    Ok(ImportedRecording {
        times,
        robots: out_robots,
        object_tracks,
        warnings,
    })
}

/// Where a robot lives in the recording: an explicit
/// [`RecordingImportOptions::robot_roots`] entry, else the export
/// convention `/World/<sanitized name>`, else (sole robot only) the given
/// structural search.
fn resolve_stage_root(
    opened: &OpenedStage,
    options: &RecordingImportOptions,
    name: &str,
    sole: bool,
    search: impl FnOnce() -> Result<String, UsdExportError>,
) -> Result<String, UsdImportError> {
    if let Some((_, root)) = options.robot_roots.iter().find(|(n, _)| n == name) {
        if !opened.has_prim(root) {
            return Err(UsdImportError::Recording(format!(
                "robot root `{root}` (for `{name}`) not found in the recording"
            )));
        }
        return Ok(root.clone());
    }
    let conventional = format!("/World/{}", sanitize_name(name));
    if opened.has_prim(&conventional) {
        Ok(conventional)
    } else if sole {
        search().map_err(|e| UsdImportError::Recording(e.to_string()))
    } else {
        Err(UsdImportError::Recording(format!(
            "cannot locate robot `{name}` in the recording (no `{conventional}`); pass \
             robot_roots with its prim path"
        )))
    }
}

/// The exporter's `/World/Env` prim path for an obstacle name.
fn env_path_of(name: &str) -> String {
    let mut path = "/World/Env".to_string();
    for seg in name.split('/').filter(|s| !s.is_empty()) {
        path.push('/');
        path.push_str(&sanitize_name(seg));
    }
    path
}

fn value_to_f64(v: &sdf::Value) -> Option<f64> {
    match v {
        sdf::Value::Double(d) => Some(*d),
        sdf::Value::Float(f) => Some(*f as f64),
        sdf::Value::Int(i) => Some(*i as f64),
        sdf::Value::Int64(i) => Some(*i as f64),
        _ => None,
    }
}

/// Min/max authored sample codes over the tracked prims' xformOps.
fn scan_sample_range(stage: &Stage, paths: &[String]) -> (Option<f64>, Option<f64>) {
    let (mut lo, mut hi) = (None::<f64>, None::<f64>);
    for path in paths {
        let Ok(prim_path) = sdf::path(path) else {
            continue;
        };
        let view = AnyPrim(stage.prim(prim_path));
        let Ok(Some(order)) = view.xform_op_order() else {
            continue;
        };
        for op in order {
            let Ok(attr_path) = view.prim().path().append_property(op.as_str()) else {
                continue;
            };
            let Ok(Some(sample_times)) = stage.time_sample_times(attr_path) else {
                continue;
            };
            for t in sample_times {
                lo = Some(lo.map_or(t, |v: f64| v.min(t)));
                hi = Some(hi.map_or(t, |v: f64| v.max(t)));
            }
        }
    }
    (lo, hi)
}

/// Root→prim chains for cheap per-frame world evaluation (only tracked
/// prims and their ancestors get touched, not the whole stage).
fn build_chains(stage: &Stage, paths: &[String]) -> Result<Vec<Vec<Prim>>, UsdImportError> {
    paths
        .iter()
        .map(|path| {
            let mut chain = Vec::new();
            let mut acc = String::new();
            for part in path.split('/').filter(|p| !p.is_empty()) {
                acc.push('/');
                acc.push_str(part);
                let prim_path = sdf::path(&acc)
                    .map_err(|e| UsdImportError::Recording(format!("bad prim path: {e}")))?;
                chain.push(stage.prim(prim_path));
            }
            if chain.is_empty() {
                return Err(UsdImportError::Recording(format!(
                    "prim `{path}` not found in the recording"
                )));
            }
            Ok(chain)
        })
        .collect()
}

/// Raw stage-space world transform per tracked prim at one time code
/// (row-vector convention: world = local · parent), plus the chain's
/// accumulated uniform scale (cube root of the residual determinant).
fn evaluate_chains(chains: &[Vec<Prim>], code: f64) -> Vec<(Isometry3<f64>, f64)> {
    chains
        .iter()
        .map(|chain| {
            let mut world = gf::Matrix4d::default();
            for prim in chain {
                let view = AnyPrim(prim.clone());
                if let Ok(local) = view.local_to_parent_transform(TimeCode::new(code)) {
                    world = local * world;
                }
            }
            let (pose, residual) = decompose_matrix(&world);
            let det = residual.determinant();
            let scale = if det.is_finite() && det > 1e-12 {
                det.cbrt()
            } else {
                1.0
            };
            (pose, scale)
        })
        .collect()
}

/// Reads q(t) from `JointStateAPI`. Joint mode needs *every* actuated
/// joint readable over the whole range; a partial set gets a warning and
/// falls back to transforms.
fn read_joint_states(
    stage: &Stage,
    model: &RobotModel,
    info: &RobotStageInfo,
    stage_root: &str,
    model_root: &str,
    codes: &[f64],
    warnings: &mut Vec<String>,
) -> Option<Vec<Vec<f64>>> {
    let dof = model.dof();
    if dof == 0 {
        return None;
    }
    let mut tracks: Vec<(usize, Vec<f64>)> = Vec::new();
    for joint in &model.joints {
        let Some(qi) = joint.q_index else { continue };
        let (instance, from_stage): (&str, fn(f64, f64) -> f64) = match joint.joint_type {
            JointType::Revolute | JointType::Continuous => ("angular", |v, _| v.to_radians()),
            JointType::Prismatic => ("linear", |v, mpu| v * mpu),
            JointType::Fixed => continue,
        };
        let prim_path = remap_model_path(stage_root, model_root, &joint.name);
        let attr_path = sdf::path(&prim_path).ok().and_then(|p| {
            p.append_property(format!("state:{instance}:physics:position").as_str())
                .ok()
        });
        let Some(attr_path) = attr_path else { continue };
        let mut track = Vec::with_capacity(codes.len());
        for &code in codes {
            match stage
                .attribute(attr_path.clone())
                .get_at::<sdf::Value>(TimeCode::new(code))
            {
                // The usda parser widens float samples to Double on read;
                // accept both (and integer-authored states).
                Ok(Some(v)) => match value_to_f64(&v) {
                    Some(raw) => track.push(from_stage(raw, info.frame.mpu)),
                    None => break,
                },
                _ => break,
            }
        }
        if track.len() == codes.len() {
            tracks.push((qi, track));
        }
    }
    if tracks.is_empty() {
        return None;
    }
    if tracks.len() != dof {
        warnings.push(format!(
            "JointState covers only {}/{dof} actuated joints; playing transforms instead",
            tracks.len()
        ));
        return None;
    }
    let mut samples = vec![vec![0.0; dof]; codes.len()];
    for (qi, track) in tracks {
        for (f, v) in track.into_iter().enumerate() {
            samples[f][qi] = v;
        }
    }
    Some(samples)
}

/// Joint-state sanity: FK from q(t) — with the recorded root pose as the
/// base, so mobile bases replay too — must land on the recorded body
/// transforms. Sampled at the first/middle/last frames.
fn check_fk_residual(
    model: &RobotModel,
    link_poses: &[Vec<Isometry3<f64>>],
    joint_samples: &[Vec<f64>],
    warnings: &mut Vec<String>,
) {
    let n = link_poses.len();
    let mut worst = 0.0f64;
    for f in [0, n / 2, n.saturating_sub(1)] {
        let base = link_poses[f][model.root_link];
        let Ok(fk) = forward_kinematics_with_base(model, &joint_samples[f], &base) else {
            continue;
        };
        for (a, b) in fk.iter().zip(&link_poses[f]) {
            worst = worst.max((a.translation.vector - b.translation.vector).norm());
        }
    }
    if worst > 1e-3 {
        warnings.push(format!(
            "joint-state FK deviates from the recorded transforms by up to {:.1} mm — the \
             recording may not match this robot model",
            worst * 1e3
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::articulation::{import_robot, RobotImportOptions};
    use crate::export::{
        write_animation, AnimationInput, ExportOptions, ObjectSpec, PoseTrack, RobotAnimation,
        TEST_ARM,
    };
    use botrail_model::Geometry;
    use nalgebra::{UnitQuaternion, Vector3};

    fn temp_dir(tag: &str) -> PathBuf {
        static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "botrail-usd-recording-{tag}-{}-{n}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    struct Fixture {
        model: RobotModel,
        anim: PathBuf,
        times: Vec<f64>,
        configs: Vec<Vec<f64>>,
        link_poses: Vec<Vec<Isometry3<f64>>>,
        box_track: Vec<Isometry3<f64>>,
    }

    /// Import the ARM stage, bake a densely-sampled FK animation (with
    /// JointState and one moving + one static obstacle), and write it out —
    /// the recording under test.
    fn export_fixture(usda: &str, tag: &str) -> Fixture {
        let dir = temp_dir(tag);
        std::fs::write(dir.join("robot.usda"), usda).unwrap();
        let imported = import_robot(
            &dir.join("robot.usda"),
            &RobotImportOptions {
                mesh_cache_dir: Some(dir.join("meshes")),
                ..Default::default()
            },
        )
        .unwrap();
        write_fixture(imported.model, dir)
    }

    /// The same recording baked from a URDF arm — a robot with no USD
    /// stage behind it, exported the flat per-link way.
    fn baked_fixture(tag: &str) -> Fixture {
        let model = RobotModel::from_urdf_str(TEST_URDF_ARM).unwrap();
        write_fixture(model, temp_dir(tag))
    }

    const TEST_URDF_ARM: &str = r#"
        <robot name="baked_arm">
          <link name="base_link"/>
          <link name="upper"/>
          <link name="tip"/>
          <joint name="j1" type="revolute">
            <parent link="base_link"/><child link="upper"/>
            <origin xyz="0 0 0.2"/><axis xyz="0 0 1"/>
            <limit lower="-3" upper="3" effort="10" velocity="1"/>
          </joint>
          <joint name="j2" type="prismatic">
            <parent link="upper"/><child link="tip"/>
            <origin xyz="0.1 0 0.1"/><axis xyz="1 0 0"/>
            <limit lower="-0.5" upper="1.5" effort="10" velocity="1"/>
          </joint>
        </robot>
    "#;

    fn write_fixture(model: RobotModel, dir: PathBuf) -> Fixture {
        let base = Isometry3::from_parts(
            Translation3::new(0.3, -0.2, 0.1),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.4),
        );
        // One authored sample per integer frame at 60 fps — the shape real
        // recorders produce, and exactly the grid the importer walks.
        let times: Vec<f64> = (0..=24).map(|k| k as f64 / 60.0).collect();
        let configs: Vec<Vec<f64>> = times
            .iter()
            .map(|t| vec![1.2 * (3.0 * t).sin(), -0.4 + 0.9 * t])
            .collect();
        let link_poses: Vec<Vec<Isometry3<f64>>> = configs
            .iter()
            .map(|q| forward_kinematics_with_base(&model, q, &base).unwrap())
            .collect();

        let box_track: Vec<Isometry3<f64>> = times
            .iter()
            .map(|t| {
                Isometry3::from_parts(
                    Translation3::new(0.5 + 0.15 * t, 0.1, 0.05),
                    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5 * t),
                )
            })
            .collect();
        let objects = [
            ObjectSpec {
                name: "box".into(),
                geometry: Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                track: PoseTrack::Sampled(box_track.clone()),
                color: None,
                visible: Vec::new(),
            },
            ObjectSpec {
                name: "table".into(),
                geometry: Geometry::Box {
                    size: Vector3::new(1.0, 1.0, 0.02),
                },
                track: PoseTrack::Static(Isometry3::translation(0.5, 0.0, -0.01)),
                color: None,
                visible: Vec::new(),
            },
        ];

        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &link_poses,
            joint_samples: Some(&configs),
        }];
        let input = AnimationInput {
            robots: &robots,
            times: &times,
            objects: &objects,
        };
        let anim = dir.join("anim.usda");
        let warnings = write_animation(&anim, &input, &ExportOptions::default()).unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");
        Fixture {
            model,
            anim,
            times,
            configs,
            link_poses,
            box_track,
        }
    }

    fn assert_pose_close(a: &Isometry3<f64>, b: &Isometry3<f64>, tol: f64, ctx: &str) {
        let dt = (a.translation.vector - b.translation.vector).norm();
        let dr = a.rotation.angle_to(&b.rotation);
        assert!(
            dt < tol && dr < tol,
            "{ctx}: dt = {dt:.2e}, dr = {dr:.2e}\n  a = {a:?}\n  b = {b:?}"
        );
    }

    fn check_link_poses(fx: &Fixture, rec: &ImportedRecording, tol: f64, tag: &str) {
        assert_eq!(rec.times.len(), fx.times.len());
        for (t, expect) in rec.times.iter().zip(&fx.times) {
            assert!((t - expect).abs() < 1e-12);
        }
        for (f, poses) in rec.robots[0].link_poses.iter().enumerate() {
            for (i, pose) in poses.iter().enumerate() {
                assert_pose_close(
                    pose,
                    &fx.link_poses[f][i],
                    tol,
                    &format!("{tag} frame {f} link {}", fx.model.links[i].name),
                );
            }
        }
    }

    #[test]
    fn joint_state_roundtrip() {
        let fx = export_fixture(TEST_ARM, "joint");
        let rec = import_recording(
            &fx.anim,
            &[("Robot".to_string(), &fx.model)],
            &["box".into(), "table".into()],
            &RecordingImportOptions::default(),
        )
        .unwrap();
        assert!(rec.warnings.is_empty(), "{:?}", rec.warnings);
        assert_eq!(rec.robots[0].mode, RecordingMode::JointState);

        // q(t) roundtrips through float32 degrees.
        let samples = rec.robots[0].joint_samples.as_ref().unwrap();
        for (f, q) in samples.iter().enumerate() {
            for (a, b) in q.iter().zip(&fx.configs[f]) {
                assert!((a - b).abs() < 1e-5, "frame {f}: {a} vs {b}");
            }
        }
        check_link_poses(&fx, &rec, 1e-6, "joint");

        // Only the moving obstacle gets a track, in world coords.
        assert_eq!(rec.object_tracks.len(), 1);
        let (name, track) = &rec.object_tracks[0];
        assert_eq!(name, "box");
        for (f, pose) in track.iter().enumerate() {
            assert_pose_close(pose, &fx.box_track[f], 1e-6, &format!("box frame {f}"));
        }
    }

    #[test]
    fn transform_fallback() {
        let fx = export_fixture(TEST_ARM, "transforms");
        let rec = import_recording(
            &fx.anim,
            &[("Robot".to_string(), &fx.model)],
            &[],
            &RecordingImportOptions {
                force_transforms: true,
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(rec.robots[0].mode, RecordingMode::Transforms);
        assert!(rec.robots[0].joint_samples.is_none());
        check_link_poses(&fx, &rec, 1e-6, "transforms");
    }

    #[test]
    fn baked_roundtrip() {
        // A URDF-sourced robot has no stage to reference, so its export is
        // the flat baked shape — and it replays: resolved by the writer's
        // link naming, transform tier only, poses landing back exactly.
        let fx = baked_fixture("baked");
        let rec = import_recording(
            &fx.anim,
            &[("Robot".to_string(), &fx.model)],
            &["box".into(), "table".into()],
            &RecordingImportOptions::default(),
        )
        .unwrap();
        assert!(rec.warnings.is_empty(), "{:?}", rec.warnings);
        assert_eq!(rec.robots[0].mode, RecordingMode::Transforms);
        assert!(rec.robots[0].joint_samples.is_none());
        check_link_poses(&fx, &rec, 1e-6, "baked");
        assert_eq!(rec.object_tracks.len(), 1);
        assert_eq!(rec.object_tracks[0].0, "box");
    }

    #[test]
    fn baked_sole_robot_is_found_structurally() {
        // The cell was rebuilt under another instance name: there is no
        // `/World/renamed` prim, but a sole robot still resolves by its
        // baked link naming — the flat-shape mirror of the referenced
        // path's link-tree search.
        let fx = baked_fixture("baked-renamed");
        let rec = import_recording(
            &fx.anim,
            &[("renamed".to_string(), &fx.model)],
            &[],
            &RecordingImportOptions::default(),
        )
        .unwrap();
        assert_eq!(rec.robots[0].mode, RecordingMode::Transforms);
        check_link_poses(&fx, &rec, 1e-6, "renamed");
    }

    /// Identical twins in one recording: the structural search cannot tell
    /// them apart, but the export convention (`/World/<name>`) resolves
    /// both, and each robot's q(t) comes back as its own.
    #[test]
    fn dual_robot_recording_roundtrip() {
        let dir = temp_dir("dual");
        std::fs::write(dir.join("robot.usda"), TEST_ARM).unwrap();
        let imported = import_robot(
            &dir.join("robot.usda"),
            &RobotImportOptions {
                mesh_cache_dir: Some(dir.join("meshes")),
                ..Default::default()
            },
        )
        .unwrap();
        let model = imported.model;

        let times: Vec<f64> = (0..=12).map(|k| k as f64 / 60.0).collect();
        let qs = |phase: f64| -> Vec<Vec<f64>> {
            times
                .iter()
                .map(|t| vec![0.9 * (3.0 * t + phase).sin(), -0.3 + 0.8 * t])
                .collect()
        };
        let (qs_a, qs_b) = (qs(0.0), qs(1.0));
        let base_a = Isometry3::translation(0.0, -0.4, 0.0);
        let base_b = Isometry3::translation(0.0, 0.4, 0.0);
        let poses = |base: &Isometry3<f64>, qs: &[Vec<f64>]| -> Vec<Vec<Isometry3<f64>>> {
            qs.iter()
                .map(|q| forward_kinematics_with_base(&model, q, base).unwrap())
                .collect()
        };
        let (poses_a, poses_b) = (poses(&base_a, &qs_a), poses(&base_b, &qs_b));
        let robots = [
            RobotAnimation {
                name: "arm_a",
                model: &model,
                link_poses: &poses_a,
                joint_samples: Some(&qs_a),
            },
            RobotAnimation {
                name: "arm_b",
                model: &model,
                link_poses: &poses_b,
                joint_samples: Some(&qs_b),
            },
        ];
        let input = AnimationInput {
            robots: &robots,
            times: &times,
            objects: &[],
        };
        let anim = dir.join("cell.usda");
        write_animation(&anim, &input, &ExportOptions::default()).unwrap();

        // Convention roots (`/World/arm_a`, `/World/arm_b`) — no explicit
        // robot_roots needed for a botrail export.
        let rec = import_recording(
            &anim,
            &[("arm_a".to_string(), &model), ("arm_b".to_string(), &model)],
            &[],
            &RecordingImportOptions::default(),
        )
        .unwrap();
        assert!(rec.warnings.is_empty(), "{:?}", rec.warnings);
        assert_eq!(rec.robots.len(), 2);
        for (out, (expect_q, expect_poses)) in rec
            .robots
            .iter()
            .zip([(&qs_a, &poses_a), (&qs_b, &poses_b)])
        {
            assert_eq!(out.mode, RecordingMode::JointState);
            let samples = out.joint_samples.as_ref().unwrap();
            for (f, q) in samples.iter().enumerate() {
                for (a, b) in q.iter().zip(&expect_q[f]) {
                    assert!((a - b).abs() < 1e-5, "{}: frame {f}: {a} vs {b}", out.name);
                }
            }
            for (f, frame) in out.link_poses.iter().enumerate() {
                for (i, pose) in frame.iter().enumerate() {
                    assert_pose_close(
                        pose,
                        &expect_poses[f][i],
                        1e-6,
                        &format!("{} frame {f} link {i}", out.name),
                    );
                }
            }
        }

        // An explicit root wins over the convention (cross-wiring the two
        // instances swaps their tracks).
        let swapped = import_recording(
            &anim,
            &[("arm_a".to_string(), &model)],
            &[],
            &RecordingImportOptions {
                robot_roots: vec![("arm_a".to_string(), "/World/arm_b".to_string())],
                ..Default::default()
            },
        )
        .unwrap();
        let q0 = &swapped.robots[0].joint_samples.as_ref().unwrap()[3];
        assert!((q0[0] - qs_b[3][0]).abs() < 1e-5, "explicit root ignored");

        // A multi-robot import without locatable roots names the fix.
        let err = import_recording(
            &anim,
            &[
                ("ghost_a".to_string(), &model),
                ("ghost_b".to_string(), &model),
            ],
            &[],
            &RecordingImportOptions::default(),
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("robot_roots"), "{err}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A centimeters / Y-up robot stage: the recording composes it through
    /// the corrective orient+scale pair, and `K` must be re-expressed in
    /// recording units before use.
    #[test]
    fn yup_cm_robot_recording() {
        let usda = TEST_ARM
            .replace("metersPerUnit = 1", "metersPerUnit = 0.01")
            .replace("upAxis = \"Z\"", "upAxis = \"Y\"");
        let fx = export_fixture(&usda, "yupcm");
        for force in [false, true] {
            let rec = import_recording(
                &fx.anim,
                &[("Robot".to_string(), &fx.model)],
                &[],
                &RecordingImportOptions {
                    force_transforms: force,
                    ..Default::default()
                },
            )
            .unwrap();
            assert!(rec.warnings.is_empty(), "{:?}", rec.warnings);
            assert_eq!(
                rec.robots[0].mode,
                if force {
                    RecordingMode::Transforms
                } else {
                    RecordingMode::JointState
                }
            );
            check_link_poses(&fx, &rec, 1e-6, "yupcm");
        }
    }
}
