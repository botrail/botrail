//! Mechanical attachment checks read from the immutable assembly source: the
//! parts a product's installation documents require between it and the robot
//! flange, and the hosts a purchase kit is documented for. Nothing is inferred
//! from geometry — a passing item repeats a manufacturer statement about the
//! products actually loaded.

use std::collections::BTreeMap;

use botrail_model::{mounting::*, CatalogMeta, MountRole, RobotModel, RobotSource};
use serde::Serialize;
use serde_json::{json, Value};

use crate::{wire::PoseMsg, Scene};

mod kit;

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
    /// No item failed or stayed unknown.
    pub ready: bool,
    pub assemblies: Vec<Assembly>,
    pub items: Vec<MountingItem>,
    /// One entry per purchase kit: its host and the manufacturer's claim.
    pub kits: Vec<Value>,
}

struct Part<'a> {
    name: String,
    source: &'a RobotSource,
    meta: CatalogMeta,
}

impl Part<'_> {
    fn catalog(&self) -> Option<(&str, &str, &CatalogMeta)> {
        match self.source {
            RobotSource::Catalog {
                id, revision, meta, ..
            } => Some((id, revision, meta)),
            _ => None,
        }
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

    /// The declaration cites a manufacturer, standard or measured source.
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
    kits: Vec<KitRecord<'a>>,
}

struct KitRecord<'a> {
    name: String,
    source: &'a RobotSource,
    root: Option<usize>,
    mount_frame: Option<String>,
    members: std::ops::Range<usize>,
}

impl<'a> Graph<'a> {
    fn visit(&mut self, source: &'a RobotSource, name: &str, count: &mut usize) -> Links {
        match source {
            RobotSource::Catalog {
                meta, inner, mount, ..
            } if meta.kit.is_some() => {
                let start = self.parts.len();
                let links = self.visit(inner, &format!("{name}/components"), &mut 0);
                let root = mount.as_ref().and_then(|m| links.get(m)).map(|p| p.0);
                let mount_frame = mount
                    .as_ref()
                    .and_then(|m| links.get(m))
                    .map(|p| p.1.clone());
                self.kits.push(KitRecord {
                    name: name.into(),
                    source,
                    root,
                    mount_frame,
                    members: start..self.parts.len(),
                });
                links
            }
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
                    meta: match source {
                        RobotSource::Catalog { meta, .. } => meta.clone(),
                        _ => CatalogMeta::default(),
                    },
                });
                leaf_links(source)
                    .into_iter()
                    .map(|link| (link.clone(), (index, link)))
                    .collect()
            }
        }
    }

    /// The parts between a tool and the robot base, nearest first, and
    /// whether every step of that path resolved to a known part.
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
        // A USD package exposes its declared frames above; an undeclared USD
        // frame cannot be assigned to a part without reimporting the asset.
        _ => Vec::new(),
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
    evaluate(graph)
}

/// Inspect every robot in the scene.
pub fn report(scene: &Scene) -> MountingReport {
    let mut graph = Graph::default();
    for robot in scene.robots() {
        graph.visit(&robot.model.source, &robot.name, &mut 0);
    }
    evaluate(graph)
}

fn evaluate(graph: Graph<'_>) -> MountingReport {
    let mut report = MountingReport {
        ready: false,
        assemblies: Vec::new(),
        items: Vec::new(),
        kits: Vec::new(),
    };
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
        let Some(tool) = edge.tool.map(|i| &graph.parts[i]) else {
            continue;
        };
        let requirements: Vec<_> = tool
            .meta
            .mounting
            .as_ref()
            .map(|m| {
                m.requirements
                    .iter()
                    .filter(|r| r.frame == edge.mount)
                    .collect()
            })
            .unwrap_or_default();
        for req in requirements {
            let order = tool
                .meta
                .order
                .as_ref()
                .and_then(|o| req.order_requires.and_then(|i| o.requires.get(i)));
            let what = order
                .and_then(|r| {
                    r.part_number
                        .clone()
                        .or_else(|| r.catalog.clone())
                        .or_else(|| r.category.clone())
                })
                .unwrap_or_else(|| req.id.clone());
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
            let mut unconfirmed_alternatives = Vec::new();
            for alternative in &req.catalog_alternatives {
                for &index in &path {
                    if graph.parts[index]
                        .catalog()
                        .is_some_and(|(id, _, _)| id == alternative.catalog)
                        && !found.contains(&index)
                    {
                        if tool.supported(&alternative.evidence) {
                            found.push(index);
                        } else {
                            unconfirmed_alternatives.push(index);
                        }
                    }
                }
            }
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
            // Count each physical instance once even if several declarations
            // name it. Missing evidence for a present candidate stays unknown.
            unconfirmed_alternatives.retain(|i| !found.contains(i));
            unconfirmed_alternatives.sort_unstable();
            unconfirmed_alternatives.dedup();
            let quantity = order.map_or(1, |r| r.qty as usize);
            let status = if (order.is_some_and(|r| r.catalog.is_some())
                || !req.catalog_alternatives.is_empty()
                || req.interface_pair.is_some())
                && tool.supported(&req.evidence)
            {
                if found.len() >= quantity {
                    "pass"
                } else if found.len() + unconfirmed_alternatives.len() >= quantity {
                    "unknown"
                } else if complete_path {
                    "fail"
                } else {
                    "unknown"
                }
            } else {
                "unknown"
            };
            let message = match status {
                "pass" => format!("{what} is on the attachment path"),
                "fail" => format!(
                    "{what} is required between this tool and the robot flange but is not on the attachment path — {}",
                    req.note
                ),
                _ => format!(
                    "{what}: presence on the attachment path could not be established — {}",
                    req.note
                ),
            };
            report.add(&edge.target, &format!("required:{}", req.id), status, message,
                if status == "pass" { "" } else { "Attach the required product — or the kit that includes it — between the robot flange and this tool" },
                json!({"requirement": req, "order_requirement": order, "sources": tool.evidence(&req.evidence),
                    "alternative_sources": req.catalog_alternatives.iter().map(|a| json!({"catalog": a.catalog, "sources": tool.evidence(&a.evidence)})).collect::<Vec<_>>(),
                    "unconfirmed_alternatives": unconfirmed_alternatives.iter().map(|&p| &graph.parts[p].name).collect::<Vec<_>>(),
                    "upstream": path.iter().map(|&p| json!({"target": graph.parts[p].name, "catalog": graph.parts[p].catalog().map(|(id, rev, _)| json!({"id": id, "revision": rev}))})).collect::<Vec<_>>(),
                    "path_complete": complete_path, "found": found.iter().map(|&p| &graph.parts[p].name).collect::<Vec<_>>()}));
        }
    }
    for kit in &graph.kits {
        kit::review(kit, &graph, &mut report);
    }
    report.ready = !report
        .items
        .iter()
        .any(|i| matches!(i.status, "fail" | "unknown"));
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
