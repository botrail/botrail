//! Connection conditions for individually purchased tools and declared adapters.
use super::*;
use botrail_model::compatibility::ConnectionSelection;

fn identity_matches(part: &Part<'_>, pins: &[&PartEntry]) -> bool {
    pins.iter().filter(|p| p.target == part.name).all(|p| {
        p.part.catalog.as_ref().is_none_or(|c| {
            part.catalog().is_some_and(|(id, revision, _)| {
                c.id == id && c.revision.as_deref().is_none_or(|r| r == revision)
            })
        })
    })
}

fn mount_matches(graph: &Graph<'_>, edge: &Edge) -> bool {
    let (Some(base), Some(tool)) = (edge.base, edge.tool) else {
        return false;
    };
    let base = &graph.parts[base];
    let tool = &graph.parts[tool];
    let RobotSource::Catalog {
        flange: Some(flange),
        ..
    } = super::kit::bare(base.source)
    else {
        return false;
    };
    let RobotSource::Catalog {
        mount: Some(mount), ..
    } = super::kit::bare(tool.source)
    else {
        return false;
    };
    if edge.role != MountRole::Tool || edge.flange != *flange || edge.mount != *mount {
        return false;
    }
    if let Some(face) = tool.face(mount, InterfaceRole::Mount) {
        if let Some(poses) = &face.allowed_poses {
            return tool.supported(&face.evidence)
                && poses.iter().any(|p| pose_matches(p, &edge.offset));
        }
    }
    pose_matches(
        &MountPose {
            position: [0.; 3],
            quaternion: [0., 0., 0., 1.],
        },
        &edge.offset,
    )
}

pub(super) fn review(
    graph: &Graph<'_>,
    pins: &[&PartEntry],
    selections: &[ConnectionSelection],
) -> Vec<Value> {
    graph.parts.iter().enumerate().filter_map(|(index, part)| {
        if part.meta.compatibility.connections.is_empty() || graph.kits.iter().any(|k| k.members.contains(&index)) { return None; }
        let (id, revision, _) = part.catalog()?;
        let incoming = graph.edges.iter().find(|e| e.tool == Some(index) && e.role == MountRole::Tool);
        let (path, complete) = incoming.map(|e| graph.upstream(e)).unwrap_or_default();
        // The first ancestor that is not itself mounted as a tool is the host.
        // An arm mounted to a pedestal remains the host; the pedestal is not a coupling.
        let adapter_count = path.iter().take_while(|&&i| graph.edges.iter().any(|e| e.tool == Some(i) && e.role == MountRole::Tool)).count();
        let host = path.get(adapter_count).map(|&i| &graph.parts[i]);
        let host_id = host.and_then(|h| h.catalog()).map(|(id, _, _)| id);
        let via: Vec<_> = path[..adapter_count].iter().rev().map(|&i| graph.parts[i].catalog().map(|(id, _, _)| id)).collect();
        let selected = selections.iter().find(|s| s.target == part.name);
        let profile = selected.and_then(|s| part.meta.compatibility.connections.iter().find(|p| p.id == s.profile));
        let mut connection = part.meta.compatibility.review(id, host_id, selected, &part.meta.sources);
        let identity_ok = identity_matches(part, pins) && path.iter().take(adapter_count+1).all(|&i| identity_matches(&graph.parts[i], pins));
        let poses_ok = incoming.is_some_and(|e| mount_matches(graph, e)) && path[..adapter_count].iter().all(|&i| {
            graph.edges.iter().find(|e| e.tool == Some(i)).is_some_and(|e| mount_matches(graph, e))
        });
        let via_ok = profile.is_none_or(|p| p.via.iter().map(|id| Some(id.as_str())).eq(via.iter().copied()));
        // Fail a changed actual assembly independently of typed user declarations.
        if selected.is_some() && (!complete || host_id.is_none() || !identity_ok || !poses_ok || !via_ok) {
            connection["configuration"] = json!({"status":"fail", "message":"Actual host, adapter chain, mounting frames, pose or assigned identity differs from the documented route"});
            for scope in ["electrical", "communication", "software"] {
                connection[scope] = json!({"status":"not_applicable", "checks":[]});
            }
        }
        Some(json!({"target":part.name,"catalog":id,"revision":revision,"name":part.meta.product,
            "host":host_id,"via":via,"order":part.meta.order,"connection":connection}))
    }).collect()
}
