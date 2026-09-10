use super::*;
use botrail_model::kit::KitSpec;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

fn catalog_id(source: &RobotSource) -> Option<&str> {
    match source {
        RobotSource::Catalog { id, .. } => Some(id),
        _ => None,
    }
}

fn frame_matches(actual: &str, declared: &str) -> bool {
    if actual == declared {
        return true;
    }
    let (prefix, leaf) = declared.rsplit_once('/').unwrap_or(("", declared));
    actual.rsplit('/').next() == Some(leaf)
        && (prefix.is_empty() || actual.starts_with(&format!("{prefix}/")))
}

/// The loaded components sit on the kit's declared frames, in its declared
/// order and offsets — the assembly the manufacturer's part number names.
fn composition(source: &RobotSource, kit: &KitSpec) -> bool {
    let mut source = source;
    for a in kit.attachments.iter().rev() {
        let RobotSource::Composite {
            base,
            tool,
            flange,
            mount,
            offset,
            prefix,
            role,
            ..
        } = source
        else {
            return false;
        };
        let [r, p, y] = a.offset.rpy;
        let expected = Isometry3::from_parts(
            Translation3::from(a.offset.xyz),
            UnitQuaternion::from_euler_angles(r, p, y),
        );
        if catalog_id(tool) != Some(a.package.as_str())
            || !frame_matches(flange, &a.parent_frame)
            || !frame_matches(mount, &a.mount_frame)
            || prefix.as_deref() != Some(&format!("{}/", a.alias))
            || *role != MountRole::Tool
            || !pose_matches(
                &MountPose {
                    position: a.offset.xyz,
                    quaternion: PoseMsg::from(&expected).quaternion,
                },
                &PoseMsg::from(offset),
            )
        {
            return false;
        }
        source = base;
    }
    catalog_id(source) == Some(kit.base.as_str())
}

/// A kit claim covers the registered host flange and kit mount, using either
/// an explicitly permitted pose or the default coincident mounting frames.
fn host_mount_matches(record: &KitRecord<'_>, graph: &Graph<'_>, edge: &Edge) -> bool {
    let Some(base) = edge.base.map(|i| &graph.parts[i]) else {
        return false;
    };
    let RobotSource::Catalog {
        flange: Some(flange),
        ..
    } = base.source
    else {
        return false;
    };
    let Some(mount_frame) = &record.mount_frame else {
        return false;
    };
    if edge.role != MountRole::Tool || *flange != edge.flange || *mount_frame != edge.mount {
        return false;
    }
    let tool = &graph.parts[record.root.expect("host edge has a kit root")];
    if let Some(face) = tool.face(&edge.mount, InterfaceRole::Mount) {
        if let Some(poses) = &face.allowed_poses {
            return tool.supported(&face.evidence)
                && poses.iter().any(|pose| pose_matches(pose, &edge.offset));
        }
    }
    pose_matches(
        &MountPose {
            position: [0.0; 3],
            quaternion: [0.0, 0.0, 0.0, 1.0],
        },
        &edge.offset,
    )
}

pub(super) fn review(record: &KitRecord<'_>, graph: &Graph<'_>, report: &mut MountingReport) {
    let RobotSource::Catalog {
        id,
        revision,
        meta,
        inner,
        ..
    } = record.source
    else {
        return;
    };
    let Some(kit) = &meta.kit else {
        return;
    };
    let matches = composition(inner, kit);
    report.add(
        &record.name,
        "kit_composition",
        if matches { "pass" } else { "fail" },
        if matches {
            "Loaded components and transforms match the purchase kit"
        } else {
            "Loaded assembly differs from the declared kit"
        },
        if matches {
            ""
        } else {
            "Load the kit from the catalog as one product instead of reassembling its parts"
        },
        json!({"catalog": id, "kit": kit}),
    );
    let host_edge = record
        .root
        .and_then(|root| graph.edges.iter().find(|e| e.tool == Some(root)))
        .filter(|e| e.base.is_some_and(|i| !record.members.contains(&i)));
    let host = host_edge.and_then(|e| e.base).map(|i| &graph.parts[i]);
    let host_id = host.and_then(|h| h.catalog()).map(|(id, _, _)| id);
    let claim = host_id.and_then(|host| kit.manufacturer_support.iter().find(|s| s.host == host));
    let mounted = matches && host_edge.is_some_and(|edge| host_mount_matches(record, graph, edge));
    let (status, message) = match (host_edge, host_id, claim) {
        (None, _, _) => ("unknown", "The kit is not attached to a robot".to_string()),
        (Some(_), None, _) => (
            "unknown",
            "The kit's host is not a catalog product, so the manufacturer's installation claim cannot be matched to it".to_string(),
        ),
        (Some(_), Some(host), None) => (
            "unknown",
            format!("No manufacturer document covers this kit on {host}; absence is not incompatibility"),
        ),
        (Some(_), Some(host), Some(_)) if mounted => (
            "pass",
            format!("The manufacturer documents installation of this kit on {host}"),
        ),
        (Some(_), Some(host), Some(_)) => (
            "unknown",
            format!("The manufacturer documents this kit on {host}, but the assembly or its mounting frames differ from the documented configuration"),
        ),
    };
    let evidence = claim.map(|c| {
        json!({"scope": c.scope, "note": c.note,
            "evidence": c.evidence.iter().map(|e| json!({"source": meta.sources.get(e.source), "section": e.section})).collect::<Vec<_>>()})
    });
    report.add(
        &record.name,
        "kit_host",
        status,
        message.clone(),
        if status == "pass" {
            ""
        } else {
            "Mount the kit on a host the manufacturer documents it for, at the documented frames"
        },
        json!({"host": host_id, "claim": evidence}),
    );
    report.kits.push(json!({
        "target": record.name, "catalog": id, "revision": revision, "name": meta.product,
        "host": host_id, "order": meta.order,
        "manufacturer_support": {"status": status, "message": message,
            "scope": claim.map(|c| c.scope), "note": claim.and_then(|c| c.note.clone()),
            "evidence": evidence.and_then(|e| e.get("evidence").cloned()).unwrap_or(Value::Array(Vec::new()))},
        "composition": {"status": if matches { "pass" } else { "fail" }},
    }));
}
