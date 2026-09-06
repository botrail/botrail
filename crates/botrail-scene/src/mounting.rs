//! Mechanical declaration checks derived from the immutable assembly source.
//! No CAD fit, fastener strength, collision, or hardware certification is inferred.

use std::collections::BTreeMap;

use botrail_model::{mounting::*, CatalogMeta, MountRole, RobotModel, RobotSource};
use serde::Serialize;
use serde_json::{json, Value};

use crate::{
    part::{PartEntry, PartTargetKind},
    wire::PoseMsg,
    Scene,
};

mod fit;

pub const VALIDATOR_VERSION: &str = "mounting/2";

#[derive(Debug, Serialize)]
pub struct MountingItem {
    pub id: String,
    pub group: &'static str,
    pub target: String,
    pub status: &'static str,
    pub message: String,
    pub basis: &'static str,
    pub next_action: String,
    pub evidence: Value,
    pub required: bool,
}

#[derive(Debug, Serialize)]
pub struct Assembly {
    pub target: String,
    pub base: Option<String>,
    pub flange: String,
    pub mount: String,
    pub offset: PoseMsg,
    pub upstream_parts: Vec<String>,
}

#[derive(Debug, Serialize)]
pub struct MountingReport {
    pub scope: &'static str,
    pub validator_version: &'static str,
    /// Fingerprint of evaluated declarations/topology, not a security digest.
    pub input_hash: String,
    pub ready: bool,
    pub assemblies: Vec<Assembly>,
    pub items: Vec<MountingItem>,
}

struct Part<'a> {
    name: String,
    source: &'a RobotSource,
    meta: CatalogMeta,
}

impl Part<'_> {
    fn catalog(&self) -> Option<(&str, &str, &CatalogMeta)> {
        fn find(source: &RobotSource) -> Option<(&str, &str, &CatalogMeta)> {
            match source {
                RobotSource::Catalog {
                    id, revision, meta, ..
                } => Some((id, revision, meta)),
                RobotSource::Mounting { base, .. } | RobotSource::Visuals { base, .. } => {
                    find(base)
                }
                _ => None,
            }
        }
        find(self.source)
    }

    fn face(&self, frame: &str, role: InterfaceRole) -> Option<&MountInterface> {
        self.meta
            .mounting
            .as_ref()?
            .interfaces
            .iter()
            .find(|f| f.frame == frame && f.role == role)
    }

    fn evidence(&self, refs: &[MountEvidence]) -> Value {
        let sources = &self.meta.sources;
        json!(refs
            .iter()
            .map(|e| json!({"section": e.section, "source_index": e.source,
            "source": sources.get(e.source)}))
            .collect::<Vec<_>>())
    }

    fn supported(&self, refs: &[MountEvidence]) -> bool {
        refs.iter().any(|e| {
            self.meta.sources.get(e.source).is_some_and(|s| {
                !s.url.trim().is_empty()
                    && !e.section.trim().is_empty()
                    && matches!(
                        s.kind.as_str(),
                        "manufacturer_datasheet"
                            | "manufacturer_cad"
                            | "standard"
                            | "official_oss"
                            | "user_drawing"
                            | "measurement"
                    )
            })
        })
    }
}

struct Edge {
    base: Option<usize>,
    tool: Option<usize>,
    target: String,
    flange: String,
    mount: String,
    offset: PoseMsg,
    role: MountRole,
}

// Link ownership retains each leaf's original frame name through arbitrary
// prefixes. Names of part instances follow the existing BOM traversal.
type Links = BTreeMap<String, (usize, String)>;

#[derive(Default)]
struct Graph<'a> {
    parts: Vec<Part<'a>>,
    edges: Vec<Edge>,
}

impl<'a> Graph<'a> {
    fn visit(&mut self, source: &'a RobotSource, name: &str, count: &mut usize) -> Links {
        match source {
            RobotSource::Visuals { base, .. } => self.visit(base, name, count),
            RobotSource::Composite {
                base,
                tool,
                flange,
                mount,
                offset,
                prefix,
                role,
                group,
                ..
            } => {
                let mut links = self.visit(base, name, count);
                let tool_name = if *role == MountRole::Arm {
                    let arm = group
                        .as_deref()
                        .or_else(|| prefix.as_deref().map(|p| p.trim_end_matches('_')))
                        .unwrap_or("arm");
                    format!("{name}/{arm}")
                } else {
                    *count += 1;
                    if *count == 1 {
                        format!("{name}/tool")
                    } else {
                        format!("{name}/tool{count}")
                    }
                };
                let tool_links = self.visit(tool, &tool_name, count);
                let parent = links.get(flange);
                let child = tool_links.get(mount);
                self.edges.push(Edge {
                    base: parent.map(|p| p.0),
                    tool: child.map(|p| p.0),
                    target: child
                        .map(|p| self.parts[p.0].name.clone())
                        .unwrap_or(tool_name),
                    flange: parent.map(|p| p.1.clone()).unwrap_or(flange.clone()),
                    mount: child.map(|p| p.1.clone()).unwrap_or(mount.clone()),
                    offset: PoseMsg::from(offset),
                    role: *role,
                });
                for (link, owner) in tool_links {
                    links.insert(format!("{}{link}", prefix.as_deref().unwrap_or("")), owner);
                }
                links
            }
            _ => {
                let index = self.parts.len();
                self.parts.push(Part {
                    name: name.into(),
                    source,
                    meta: effective_meta(source),
                });
                leaf_links(source)
                    .into_iter()
                    .map(|link| (link.clone(), (index, link)))
                    .collect()
            }
        }
    }

    fn upstream(&self, edge: &Edge) -> (Vec<usize>, bool) {
        let mut path = Vec::new();
        let mut complete = edge.base.is_some() && edge.tool.is_some();
        let mut parent = edge.base;
        while let Some(p) = parent {
            if path.contains(&p) {
                complete = false;
                break;
            }
            path.push(p);
            let incoming = self.edges.iter().find(|e| e.tool == Some(p));
            if incoming.is_some_and(|e| e.base.is_none()) {
                complete = false;
            }
            parent = incoming.and_then(|e| e.base);
        }
        (path, complete)
    }
}

fn leaf_links(source: &RobotSource) -> Vec<String> {
    match source {
        RobotSource::Mounting { base, document } => {
            let mut links = leaf_links(base);
            links.extend(document.mounting.interfaces.iter().map(|f| f.frame.clone()));
            links
        }
        RobotSource::Catalog {
            tcp,
            flange,
            mount,
            arms,
            meta,
            inner,
            ..
        } => {
            let mut links = leaf_links(inner);
            links.extend(tcp.iter().chain(flange).chain(mount).cloned());
            links.extend(
                arms.iter()
                    .flat_map(|a| a.flange.iter().chain(std::iter::once(&a.tip)))
                    .cloned(),
            );
            if let Some(spec) = &meta.mounting {
                links.extend(spec.interfaces.iter().map(|f| f.frame.clone()));
            }
            links
        }
        RobotSource::UrdfXml(xml) => RobotModel::from_urdf_str(xml)
            .map(|m| m.links.into_iter().map(|l| l.name).collect())
            .unwrap_or_default(),
        RobotSource::Visuals { base, .. } => leaf_links(base),
        // A USD package exposes its declared frames above; an undeclared USD
        // frame cannot be assigned to a part without reimporting the asset.
        _ => Vec::new(),
    }
}

fn effective_meta(source: &RobotSource) -> CatalogMeta {
    match source {
        RobotSource::Catalog { meta, .. } => meta.clone(),
        RobotSource::Visuals { base, .. } => effective_meta(base),
        RobotSource::Mounting { base, document } => document.apply_to(&effective_meta(base)),
        _ => CatalogMeta::default(),
    }
}

impl MountingReport {
    fn add(
        &mut self,
        target: &str,
        key: &str,
        status: &'static str,
        message: impl Into<String>,
        next: &str,
        evidence: Value,
    ) {
        self.items.push(MountingItem {
            id: format!("mounting:{target}:{key}"),
            group: "mounting",
            target: target.into(),
            status,
            message: message.into(),
            basis: "Loaded product declarations and immutable attachment source",
            next_action: next.into(),
            evidence,
            required: true,
        });
    }
}

/// Inspect a robot without creating a scene or downloading catalog packages.
pub fn report_robot(model: &RobotModel) -> MountingReport {
    let mut graph = Graph::default();
    graph.visit(&model.source, &model.name, &mut 0);
    evaluate(graph, &[])
}

/// Inspect every robot, also comparing authored BOM identity to loaded identity.
pub fn report(scene: &Scene) -> MountingReport {
    let mut graph = Graph::default();
    for robot in scene.robots() {
        graph.visit(&robot.model.source, &robot.name, &mut 0);
    }
    evaluate(graph, scene.parts())
}

fn evaluate(graph: Graph<'_>, annotations: &[PartEntry]) -> MountingReport {
    let mut report = MountingReport {
        scope: "mechanical_assembly",
        validator_version: VALIDATOR_VERSION,
        input_hash: String::new(),
        ready: false,
        assemblies: Vec::new(),
        items: Vec::new(),
    };
    let pins: Vec<_> = annotations
        .iter()
        .filter(|p| matches!(p.kind, PartTargetKind::Robot | PartTargetKind::Tool))
        .collect();
    for part in &graph.parts {
        for pin in pins.iter().filter(|p| p.target == part.name) {
            let Some(named) = &pin.part.catalog else {
                continue;
            };
            let status = match part.catalog() {
                Some((id, revision, _))
                    if id != named.id
                        || named.revision.as_deref().is_some_and(|r| r != revision) =>
                {
                    "fail"
                }
                Some(_) => "pass",
                None => "unknown",
            };
            report.add(&part.name, "identity", status, "Authored catalog identity compared with the loaded model",
                if status == "pass" { "" } else { "Load the intended product; set_part changes BOM identity only" },
                json!({"loaded": part.catalog().map(|(id, revision, _)| json!({"id": id, "revision": revision})), "authored": named}));
        }
    }
    for edge in graph.edges.iter().filter(|e| e.role == MountRole::Tool) {
        let (path, complete_path) = graph.upstream(edge);
        report.assemblies.push(Assembly {
            target: edge.target.clone(),
            base: edge.base.map(|p| graph.parts[p].name.clone()),
            flange: edge.flange.clone(),
            mount: edge.mount.clone(),
            offset: edge.offset.clone(),
            upstream_parts: path.iter().map(|&p| graph.parts[p].name.clone()).collect(),
        });
        let base = edge.base.map(|i| &graph.parts[i]);
        let tool = edge.tool.map(|i| &graph.parts[i]);
        let flange = base.and_then(|p| p.face(&edge.flange, InterfaceRole::Flange));
        let mount = tool.and_then(|p| p.face(&edge.mount, InterfaceRole::Mount));
        let documented = flange
            .zip(mount)
            .zip(base.zip(tool))
            .is_some_and(|((f, m), (b, t))| b.supported(&f.evidence) && t.supported(&m.evidence));
        let ids = flange
            .and_then(|f| f.interface_id.as_deref())
            .zip(mount.and_then(|m| m.interface_id.as_deref()));
        let status = match ids {
            Some((a, b)) if documented => {
                if a == b {
                    "pass"
                } else {
                    "fail"
                }
            }
            _ => "unknown",
        };
        let message = match (status, ids) {
            ("fail", Some((a, b))) => {
                format!("Bare mating interfaces differ: flange {a}; mount {b}")
            }
            _ => "Declared bare mating interface identifiers compared".into(),
        };
        report.add(
            &edge.target,
            "interface",
            status,
            message,
            if status == "pass" {
                ""
            } else {
                "Provide the mating-face drawings and evidence, or select a compatible adapter"
            },
            json!({"flange": flange, "mount": mount,
                "flange_sources": flange.zip(base).map(|(f, p)| p.evidence(&f.evidence)),
                "mount_sources": mount.zip(tool).map(|(f, p)| p.evidence(&f.evidence))}),
        );
        let poses = mount.and_then(|m| m.allowed_poses.as_ref());
        let pose_supported = mount
            .zip(tool)
            .is_some_and(|(m, t)| t.supported(&m.evidence));
        let pose_status = match poses {
            Some(poses) if pose_supported => {
                if poses.iter().any(|p| pose_matches(p, &edge.offset)) {
                    "pass"
                } else {
                    "fail"
                }
            }
            _ => "unknown",
        };
        report.add(&edge.target, "pose", pose_status, "Attachment transform compared with explicitly permitted poses",
            if pose_status == "pass" { "" } else { "Confirm the model frame convention and permitted mating pose; an offset does not add a physical adapter" },
            json!({"actual": edge.offset, "allowed": poses, "numerical_tolerance": 1e-8,
                "sources": mount.zip(tool).map(|(f, p)| p.evidence(&f.evidence))}));
        let requirements: Vec<_> = tool
            .and_then(|p| p.meta.mounting.as_ref())
            .map(|m| {
                m.requirements
                    .iter()
                    .filter(|r| r.frame == edge.mount)
                    .collect()
            })
            .unwrap_or_default();
        if requirements.is_empty() {
            let complete = mount
                .zip(tool)
                .is_some_and(|(m, t)| m.requirements_complete && t.supported(&m.evidence));
            report.add(&edge.target, "requirements", if complete { "pass" } else { "unknown" }, if complete { "Installation declaration requires no additional adapter" } else { "No mounting part requirements documented for this face" },
                "Confirm the required coupling, bracket and supplied parts from the installation manual", json!({}));
        } else if !mount
            .zip(tool)
            .is_some_and(|(m, t)| m.requirements_complete && t.supported(&m.evidence))
        {
            report.add(
                &edge.target,
                "requirements_coverage",
                "unknown",
                "Completeness of the required mounting parts is unconfirmed",
                "Confirm that the installation declaration covers every required part",
                json!({}),
            );
        }
        for req in requirements {
            let tool = tool.expect("requirement has a catalog source");
            let order = tool
                .meta
                .order
                .as_ref()
                .and_then(|o| req.order_requires.and_then(|i| o.requires.get(i)));
            let mut found: Vec<_> = order
                .and_then(|r| r.catalog.as_deref())
                .map(|id| {
                    path.iter()
                        .filter(|&&p| {
                            graph.parts[p]
                                .catalog()
                                .is_some_and(|(loaded, _, _)| loaded == id)
                        })
                        .copied()
                        .collect()
                })
                .unwrap_or_default();
            if let Some(pair) = &req.interface_pair {
                for (position, &index) in path.iter().enumerate() {
                    let part = &graph.parts[index];
                    let child = if position == 0 {
                        edge.tool
                    } else {
                        Some(path[position - 1])
                    };
                    let incoming = graph.edges.iter().find(|e| e.tool == Some(index));
                    let outgoing = graph
                        .edges
                        .iter()
                        .find(|e| e.base == Some(index) && e.tool == child);
                    // Both declared faces must be the faces actually used on
                    // this path. Unused compatible ports are not an adapter.
                    if incoming.zip(outgoing).is_some_and(|(a, b)| {
                        [
                            (&a.mount, InterfaceRole::Mount, &pair.mount),
                            (&b.flange, InterfaceRole::Flange, &pair.flange),
                        ]
                        .iter()
                        .all(|(frame, role, id)| {
                            part.face(frame, *role).is_some_and(|f| {
                                f.interface_id.as_ref() == Some(id) && part.supported(&f.evidence)
                            })
                        })
                    }) && !found.contains(&index)
                    {
                        found.push(index);
                    }
                }
            }
            // A category/model label or a part elsewhere in the BOM cannot
            // establish that the required product is physically in this path.
            let status = if (order.is_some_and(|r| r.catalog.is_some())
                || req.interface_pair.is_some())
                && tool.supported(&req.evidence)
            {
                if found.len() >= order.map_or(1, |r| r.qty as usize) {
                    "pass"
                } else if complete_path {
                    "fail"
                } else {
                    "unknown"
                }
            } else {
                "unknown"
            };
            report.add(&edge.target, &format!("required:{}", req.id), status, req.note.clone(),
                if status == "pass" { "" } else { "Specify and attach the required product on this tool's upstream path" },
                json!({"requirement": req, "order_requirement": order, "sources": tool.evidence(&req.evidence),
                    "upstream": path.iter().map(|&p| json!({"target": graph.parts[p].name, "catalog": graph.parts[p].catalog().map(|(id, rev, _)| json!({"id": id, "revision": rev}))})).collect::<Vec<_>>(),
                    "path_complete": complete_path, "found": found.iter().map(|&p| &graph.parts[p].name).collect::<Vec<_>>()}));
        }
        fit::review(
            &mut report,
            &edge.target,
            base,
            tool,
            flange,
            mount,
            &edge.offset,
        );
    }
    // An unmounted standalone tool still has an unresolved mounting task.
    for (index, part) in graph.parts.iter().enumerate() {
        let is_tool = match part.meta.category.as_deref() {
            Some(c) => c == "adapter" || c.starts_with("tool.") || c.starts_with("gripper."),
            None => part
                .meta
                .mounting
                .as_ref()
                .is_some_and(|m| m.interfaces.iter().any(|f| f.role == InterfaceRole::Mount)),
        };
        if is_tool && !graph.edges.iter().any(|e| e.tool == Some(index)) {
            report.add(
                &part.name,
                "unmounted",
                "unknown",
                "Tool has no declared incoming attachment",
                "Attach this product to the intended robot or adapter",
                json!({}),
            );
        }
    }
    if report.items.is_empty() {
        report.add(
            "cell",
            "none",
            "not_applicable",
            "No end-effector attachments declared",
            "",
            json!({"scope_note": "Does not detect equipment omitted from the scene"}),
        );
    }
    let inputs = json!({"validator_version": VALIDATOR_VERSION, "parts": graph.parts.iter().map(|p|
        json!({"target": p.name, "catalog": p.catalog().map(|(id, revision, _)| json!({"id": id, "revision": revision})),
            "mounting": p.meta.mounting, "order": p.meta.order, "sources": p.meta.sources,
            "document": match p.source { RobotSource::Mounting { document, .. } => Some(document), _ => None }})).collect::<Vec<_>>(),
        "assemblies": report.assemblies, "annotations": pins});
    let bytes = serde_json::to_vec(&inputs).expect("mounting inputs");
    let hash = bytes.iter().fold(0xcbf29ce484222325u64, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x100000001b3)
    });
    report.input_hash = format!("fnv1a64:{hash:016x}");
    report.ready = !report
        .items
        .iter()
        .any(|i| matches!(i.status, "fail" | "unknown" | "not_run"));
    report
}

fn pose_matches(allowed: &MountPose, actual: &PoseMsg) -> bool {
    allowed
        .position
        .iter()
        .zip(actual.position)
        .all(|(a, b)| (a - b).abs() <= 1e-8)
        && [1.0, -1.0].iter().any(|sign| {
            allowed
                .quaternion
                .iter()
                .zip(actual.quaternion)
                .all(|(a, b)| (a - sign * b).abs() <= 1e-8)
        })
}
