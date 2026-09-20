//! A vehicle's catalog frame, captured from the loaded carrier. This checks
//! frame alignment, not manufacturer approval, fasteners or load capacity.

use botrail_model::{JointType, RobotModel, RobotSource};
use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};
use serde_json::json;

use super::{pose_matches, supported_source, Edge, Graph, Links, MountingReport, Part};
use crate::{part::CatalogRef, wire::PoseMsg, Scene, SceneError};
use botrail_model::mounting::{
    CatalogOrder, CatalogSource, InterfaceRole, MountPose, MountingSpec,
};

/// Geometry-free snapshot of the carrier frame used for a vehicle mount.
/// Capturing the frame once avoids re-fetching or duplicating carrier meshes
/// when saving, cloning or generating an offline Python replay.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct VehicleMountReference {
    pub carrier: Option<CatalogRef>,
    pub sources: Vec<CatalogSource>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mounting: Option<MountingSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub order: Option<CatalogOrder>,
    /// False for older snapshots or a carrier whose internal assembly was
    /// not captured. Absence on such a path cannot prove a required part missing.
    #[serde(default)]
    pub path_complete: bool,
    pub flange: String,
    pub declared_flange: Option<String>,
    /// Root of the carrier model to the selected flange, metres / XYZW.
    pub flange_pose: PoseMsg,
    /// Actual link on the mounted robot. Resolved again on replay.
    pub mount: String,
}

/// A mount must be rigid with respect to the model root. Wheel articulation
/// on another branch is harmless; a moving mounting frame is not.
fn fixed_pose(model: &RobotModel, frame: &str) -> Result<Isometry3<f64>, SceneError> {
    let mut link = model
        .link_index(frame)
        .ok_or_else(|| SceneError::UnknownLink(frame.into()))?;
    let mut pose = Isometry3::identity();
    while let Some(index) = model.links[link].parent_joint {
        let joint = &model.joints[index];
        if joint.joint_type != JointType::Fixed {
            return Err(SceneError::BadMount(format!(
                "mounting frame `{frame}` must be fixed to the model root"
            )));
        }
        pose = joint.origin * pose;
        link = joint.parent_link;
    }
    Ok(pose)
}

fn catalog(source: &RobotSource) -> Option<(CatalogRef, Vec<CatalogSource>)> {
    match source {
        RobotSource::Catalog {
            id, revision, meta, ..
        } => Some((
            CatalogRef {
                id: id.clone(),
                revision: Some(revision.clone()),
            },
            meta.sources.clone(),
        )),
        // Tools do not change the identity of the arm's base.
        RobotSource::Composite { base, .. } => catalog(base),
        _ => None,
    }
}

/// The same mount-side allowed_poses and evidence used by tool kit review.
/// Without such a declaration this phase reviews coincident catalog frames.
fn allowed_poses(source: &RobotSource, mount: &str) -> (Vec<MountPose>, bool, serde_json::Value) {
    if let RobotSource::Composite { base, .. } = source {
        return allowed_poses(base, mount);
    }
    if let RobotSource::Catalog { meta, .. } = source {
        if let Some(face) = meta.mounting.as_ref().and_then(|m| {
            m.interfaces
                .iter()
                .find(|f| f.frame == mount && f.role == InterfaceRole::Mount)
        }) {
            if let Some(poses) = &face.allowed_poses {
                let supported = face.evidence.iter().any(|e| {
                    !e.section.trim().is_empty()
                        && meta.sources.get(e.source).is_some_and(supported_source)
                });
                return (
                    poses.clone(),
                    supported,
                    json!({"interface": face, "sources": meta.sources}),
                );
            }
        }
    }
    (
        vec![MountPose {
            position: [0.0; 3],
            quaternion: [0.0, 0.0, 0.0, 1.0],
        }],
        true,
        json!({"basis": "Coincident catalog frames; no allowed_poses declaration"}),
    )
}

fn pose_isometry(pose: &MountPose) -> Isometry3<f64> {
    Isometry3::from(&PoseMsg {
        position: pose.position,
        quaternion: pose.quaternion,
    })
}

impl VehicleMountReference {
    pub fn new(
        carrier: &RobotModel,
        robot: &RobotModel,
        flange: Option<&str>,
        mount: Option<&str>,
    ) -> Result<Self, SceneError> {
        let declared_flange = carrier.flange_link.map(|i| carrier.links[i].name.clone());
        let flange = flange.or(declared_flange.as_deref()).ok_or_else(|| {
            SceneError::BadMount(
                "carrier declares no flange frame; supply flange= to record an unverified frame"
                    .into(),
            )
        })?;
        let mount = mount.unwrap_or(&robot.links[robot.mount_link.unwrap_or(robot.root_link)].name);
        fixed_pose(robot, mount)?;
        // A composite's exposed flange may belong to an attached product;
        // do not attribute it to the base carrier's catalog identity.
        let (carrier_id, sources) = if matches!(&carrier.source, RobotSource::Catalog { .. }) {
            catalog(&carrier.source).map_or((None, Vec::new()), |(id, sources)| (Some(id), sources))
        } else {
            (None, Vec::new())
        };
        Ok(Self {
            carrier: carrier_id,
            sources,
            mounting: match &carrier.source {
                RobotSource::Catalog { meta, .. } => meta.mounting.clone(),
                _ => None,
            },
            order: match &carrier.source {
                RobotSource::Catalog { meta, .. } => meta.order.clone(),
                _ => None,
            },
            path_complete: matches!(&carrier.source, RobotSource::Catalog { meta, .. } if meta.kit.is_none()),
            flange: flange.into(),
            flange_pose: PoseMsg::from(&fixed_pose(carrier, flange)?),
            declared_flange,
            mount: mount.into(),
        })
    }

    pub fn validate(&self, robot: &RobotModel) -> Result<(), SceneError> {
        if let Some(spec) = &self.mounting {
            spec.validate(&self.sources, self.order.as_ref())
                .map_err(SceneError::BadMount)?;
        }
        let pose = &self.flange_pose;
        if self.flange.trim().is_empty()
            || self
                .declared_flange
                .as_ref()
                .is_some_and(|s| s.trim().is_empty())
            || !pose
                .position
                .iter()
                .chain(&pose.quaternion)
                .all(|v| v.is_finite())
            || (pose.quaternion.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-6
            || self.carrier.as_ref().is_some_and(|c| {
                c.id.trim().is_empty() || c.revision.as_ref().is_none_or(|r| r.trim().is_empty())
            })
        {
            return Err(SceneError::BadMount("invalid carrier mounting reference: expected a pinned identity, frame and finite unit pose".into()));
        }
        fixed_pose(robot, &self.mount)?;
        Ok(())
    }

    /// First declared pose, or coincident frames, including a non-root mount.
    pub fn aligned_offset(&self, robot: &RobotModel) -> Result<Isometry3<f64>, SceneError> {
        self.validate(robot)?;
        let (poses, _, _) = allowed_poses(&robot.source, &self.mount);
        Ok(Isometry3::from(&self.flange_pose)
            * pose_isometry(&poses[0])
            * fixed_pose(robot, &self.mount)?.inverse())
    }
}

pub(super) fn review(scene: &Scene, report: &mut MountingReport) {
    for robot in scene.robots() {
        let Some(binding) = &robot.mount else {
            continue;
        };
        // A gait makes the robot the vehicle's legs; it is not a bolted arm.
        if binding.gait.is_some() || !binding.spin.is_empty() {
            continue;
        }
        let target = &robot.name;
        let Some(reference) = &binding.reference else {
            report.add(target, "mount_pose", "unknown",
                format!("Vehicle `{}` mount has only an offset; no carrier frame was recorded", binding.device),
                "Pass the loaded carrier to mount_robot(carrier=...) to review catalog frame alignment",
                json!({"device": binding.device, "actual_offset": PoseMsg::from(&binding.offset), "scope": "catalog_frame_alignment"}));
            continue;
        };
        let expected = reference.aligned_offset(&robot.model);
        let (poses, poses_supported, pose_evidence) =
            allowed_poses(&robot.model.source, &reference.mount);
        let declared_mount =
            &robot.model.links[robot.model.mount_link.unwrap_or(robot.model.root_link)].name;
        // The model root is an exact coordinate origin, not a claim about a
        // physical mounting face. A distinct face needs source support for
        // its transform; an explicit allowed pose always needs evidence.
        let arm_root = reference.mount == robot.model.links[robot.model.root_link].name;
        let known = reference.carrier.is_some()
            && catalog(&robot.model.source)
                .is_some_and(|(_, sources)| arm_root || sources.iter().any(supported_source))
            && reference.declared_flange.as_deref() == Some(reference.flange.as_str())
            && reference.mount == *declared_mount
            && reference.sources.iter().any(supported_source)
            && poses_supported;
        let actual_face = fixed_pose(&robot.model, &reference.mount)
            .ok()
            .map(|mount| {
                Isometry3::from(&reference.flange_pose).inverse() * binding.offset * mount
            });
        let (status, message, extra) = match expected {
            Ok(_) if known => {
                let actual_face = actual_face.expect("validated mount");
                let actual = PoseMsg::from(&actual_face);
                let matched = poses.iter().find(|p| pose_matches(p, &actual));
                let selected = pose_isometry(matched.unwrap_or(&poses[0]));
                let expected = Isometry3::from(&reference.flange_pose) * selected
                    * fixed_pose(&robot.model, &reference.mount).expect("validated mount").inverse();
                let aligned = matched.is_some();
                let error = selected.inverse() * actual_face;
                let translation = error.translation.vector.norm();
                let angle = error.rotation.angle();
                (if aligned { "pass" } else { "fail" },
                 if aligned { "The arm mount matches the catalog mounting frames and reference pose".into() }
                 else { format!("The arm mount differs from the catalog reference pose by {:.3} mm and {:.3} degrees", translation * 1000.0, angle.to_degrees()) },
                 json!({"expected_offset": PoseMsg::from(&expected), "actual_offset": PoseMsg::from(&binding.offset),
                    "flange_to_mount": actual, "translation_error_m": translation, "rotation_error_rad": angle}))
            }
            _ => ("unknown", "The catalog identity, sources or declared mounting frames are missing or differ from the selected frames".into(), json!({"actual_offset": PoseMsg::from(&binding.offset)})),
        };
        report.add(target, "mount_pose", status, message,
            if status == "pass" { "" } else { "Use the declared carrier/arm frames; review any offset or adapter as a separate assembly" },
            json!({"scope": "catalog_frame_alignment", "device": binding.device, "reference": reference,
                "arm": catalog(&robot.model.source).map(|(id, _)| id), "comparison": extra,
                "arm_mount_basis": if arm_root { "model_root" } else { "declared_mount_frame" },
                "arm_sources": catalog(&robot.model.source).map(|(_, sources)| sources),
                "allowed_poses": poses, "pose_evidence": pose_evidence,
                "limits": "Frame alignment only; no claim about manufacturer-supported host combinations, fasteners or load capacity"}));
    }
}

/// Add the vehicle as the actual upstream part, so arm requirements and tool
/// requirements use the same path traversal. A BOM label is never used here.
pub(super) fn connect(robot: &crate::SceneRobot, links: &Links, graph: &mut Graph<'_>) {
    let Some(binding) = &robot.mount else {
        return;
    };
    if binding.gait.is_some() || !binding.spin.is_empty() {
        return;
    }
    let reference = binding.reference.as_ref();
    let mount = reference.map_or_else(
        || {
            robot.model.links[robot.model.mount_link.unwrap_or(robot.model.root_link)]
                .name
                .as_str()
        },
        |r| r.mount.as_str(),
    );
    let child = links.get(mount);
    let parent = graph.parts.len();
    graph.parts.push(Part {
        name: binding.device.clone(),
        identity: reference.and_then(|r| r.carrier.clone()),
        flange: reference.and_then(|r| r.declared_flange.clone()),
        path_complete: reference.is_some_and(|r| r.path_complete),
        meta: botrail_model::CatalogMeta {
            mounting: reference.and_then(|r| r.mounting.clone()),
            order: reference.and_then(|r| r.order.clone()),
            sources: reference.map_or_else(Vec::new, |r| r.sources.clone()),
            ..Default::default()
        },
    });
    let offset = reference.and_then(|r| {
        fixed_pose(&robot.model, &r.mount)
            .ok()
            .map(|m| Isometry3::from(&r.flange_pose).inverse() * binding.offset * m)
    });
    graph.edges.push(Edge {
        base: reference.map(|_| parent),
        tool: child.map(|p| p.0),
        target: child.map_or_else(|| robot.name.clone(), |p| graph.parts[p.0].name.clone()),
        flange: reference.map_or_else(String::new, |r| r.flange.clone()),
        mount: child.map_or_else(|| mount.to_string(), |p| p.1.clone()),
        offset: PoseMsg::from(&offset.unwrap_or(binding.offset)),
        role: botrail_model::MountRole::Arm,
    });
}
