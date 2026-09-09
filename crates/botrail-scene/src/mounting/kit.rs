use super::*;
use botrail_model::kit::{KitSpec, RepresentationStatus};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

pub(super) fn bare(source: &RobotSource) -> &RobotSource {
    match source {
        RobotSource::Mounting { base, .. } | RobotSource::Visuals { base, .. } => bare(base),
        _ => source,
    }
}

fn catalog_id(source: &RobotSource) -> Option<&str> {
    match bare(source) {
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

fn composition(source: &RobotSource, kit: &KitSpec) -> bool {
    let mut source = bare(source);
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
        source = bare(base);
    }
    catalog_id(source) == Some(kit.base.as_str())
}

fn visual_override(source: &RobotSource) -> bool {
    match source {
        RobotSource::Visuals { .. } => true,
        RobotSource::Mounting { base, .. } | RobotSource::Catalog { inner: base, .. } => {
            visual_override(base)
        }
        RobotSource::Composite { base, tool, .. } => visual_override(base) || visual_override(tool),
        _ => false,
    }
}

/// A kit claim covers the registered host flange and kit mount, using either
/// an explicitly permitted pose or the default coincident mounting frames.
/// The latter is a model convention, not a measured seating/TCP approval.
fn host_mount_matches(record: &KitRecord<'_>, graph: &Graph<'_>, edge: &Edge) -> bool {
    let Some(base) = edge.base.map(|i| &graph.parts[i]) else {
        return false;
    };
    let RobotSource::Catalog {
        flange: Some(flange),
        ..
    } = bare(base.source)
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

fn status<'a>(items: impl Iterator<Item = &'a str>) -> &'static str {
    let items: Vec<_> = items.collect();
    if items.contains(&"fail") {
        "fail"
    } else if items.is_empty() || items.iter().any(|s| matches!(*s, "unknown" | "not_run")) {
        "unknown"
    } else {
        "pass"
    }
}

pub(super) fn review(
    record: &KitRecord<'_>,
    graph: &Graph<'_>,
    pins: &[&PartEntry],
    configurations: &[botrail_model::compatibility::ConnectionSelection],
    report: &mut MountingReport,
) {
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
    let identity_ok = pins.iter().filter(|p| p.target == record.name).all(|p| {
        p.part
            .catalog
            .as_ref()
            .is_none_or(|c| c.id == *id && c.revision.as_ref().is_none_or(|r| r == revision))
    });
    let matches = composition(inner, kit) && identity_ok;
    report.add(
        &record.name,
        "kit_composition",
        if matches { "pass" } else { "fail" },
        if matches {
            "Loaded components and transforms match the purchase kit"
        } else {
            "Loaded assembly or assigned product differs from the declared kit"
        },
        if matches {
            ""
        } else {
            "Restore the kit components and assembly, or register a separate configuration"
        },
        json!({"catalog": id, "kit": kit}),
    );
    let host_edge = record
        .root
        .and_then(|root| graph.edges.iter().find(|e| e.tool == Some(root)))
        .filter(|e| e.base.is_some_and(|i| !record.members.contains(&i)));
    let host = host_edge.and_then(|e| e.base).map(|i| &graph.parts[i]);
    let claim = host
        .and_then(|h| h.catalog())
        .and_then(|(id, _, _)| kit.manufacturer_support.iter().find(|s| s.host == id));
    let manufacturer_support = if let Some(c) = claim {
        let mounted = host_edge.is_some_and(|edge| host_mount_matches(record, graph, edge));
        let applicable = matches && mounted;
        let connections: Vec<_> = graph
            .edges
            .iter()
            .filter(|edge| {
                applicable
                    && edge.role == MountRole::Tool
                    && edge.tool.is_some_and(|i| record.members.contains(&i))
                    && (edge.base.is_some_and(|i| record.members.contains(&i))
                        || host_edge.is_some_and(|host| std::ptr::eq(host, *edge)))
            })
            .map(|edge| &edge.target)
            .collect();
        json!({"status": if applicable { "pass" } else { "not_applicable" }, "scope": c.scope,
            "message": if applicable { "Manufacturer documents mechanical installation of this kit on this host" } else { "Assembly, host mounting frames or pose differ from the supported configuration" },
            "connections": connections, "note": c.note,
            "evidence": c.evidence.iter().map(|e| json!({"source": meta.sources.get(e.source), "section": e.section})).collect::<Vec<_>>()})
    } else {
        json!({"status": "unknown", "scope": "mechanical_mounting", "message": "No documented kit claim for the actual directly attached host; absence is not incompatibility"})
    };
    let representation = if !matches {
        "fail"
    } else if kit.representation.status == RepresentationStatus::Verified
        && !record.visual_override
        && !visual_override(inner)
    {
        "pass"
    } else {
        "unknown"
    };
    let detailed = status(
        report
            .items
            .iter()
            .filter(|i| i.target.starts_with(&format!("{}/components", record.name)))
            .map(|i| i.status),
    );
    let selected = configurations.iter().find(|s| s.target == record.name);
    let mut connection = meta.compatibility.review(
        id,
        host.and_then(|h| h.catalog()).map(|(id, _, _)| id),
        selected,
        &meta.sources,
    );
    let host_identity_ok = host.is_none_or(|h| {
        pins.iter().filter(|p| p.target == h.name).all(|p| {
            p.part.catalog.as_ref().is_none_or(|c| {
                h.catalog().is_some_and(|(id, revision, _)| {
                    c.id == id && c.revision.as_deref().is_none_or(|r| r == revision)
                })
            })
        })
    });
    let invalid_via = selected.is_some_and(|s| {
        meta.compatibility
            .connections
            .iter()
            .any(|p| p.id == s.profile && !p.via.is_empty())
    });
    if !matches
        || invalid_via
        || !host_identity_ok
        || host_edge.is_some_and(|e| !host_mount_matches(record, graph, e))
    {
        connection["configuration"] = json!({"status": "fail", "message": "Kit composition, host mounting or assigned identity differs from the declared configuration"});
        for axis in ["electrical", "communication", "software"] {
            connection[axis] = json!({"status": "not_applicable", "checks": []});
        }
    }
    report.kits.push(json!({
        "target": record.name, "catalog": id, "revision": revision, "name": meta.product,
        "host": host.and_then(|h| h.catalog()).map(|(id,_,_)| id), "order": meta.order,
        "manufacturer_support": manufacturer_support,
        "composition": {"status": if matches { "pass" } else { "fail" }},
        "model_correspondence": {"status": representation, "representation": kit.representation},
        "detailed_fit": {"status": detailed, "message": "Detailed dimensional, fastener and access checks; independent of manufacturer support"},
        "electrical": connection["electrical"],
        "connection": connection,
    }));
}
