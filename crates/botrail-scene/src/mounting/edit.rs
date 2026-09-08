//! Assembly proposals use the existing RobotSource and ProjectFile contracts.
//! A preview never edits the live scene; apply repeats the checks under its lock.

use super::*;
use crate::{
    project::{robot_source_msg, ProjectFile},
    SceneRobot,
};
use std::sync::Arc;

impl Scene {
    /// Runtime generation rejects planning jobs begun before a model swap.
    pub fn assembly_generation(&self) -> u64 {
        self.assembly_generation
    }
}

/// Versioned change fingerprint, not an authentication token. Include poses,
/// authored cell state and effective models, not just the mounting report hash.
pub fn revision(scene: &Scene) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    let mut feed = |text: &str| {
        for b in text.bytes() {
            hash = (hash ^ u64::from(b)).wrapping_mul(0x100000001b3);
        }
    };
    feed(&scene.to_project().to_json());
    feed(&scene.assembly_generation.to_string());
    for robot in scene.robots() {
        feed(&format!("{:?}", robot.model));
    }
    let mut pairs: Vec<_> = scene.inter_acm.allowed_pairs().collect();
    pairs.sort_unstable();
    feed(&format!("{pairs:?}"));
    for robot in scene.robots() {
        let mut pairs: Vec<_> = robot.acm.allowed_pairs().collect();
        pairs.sort_unstable();
        feed(&format!("{pairs:?}"));
    }
    format!("assembly/1-{hash:016x}")
}

fn joint_signature(model: &RobotModel, mut link: usize) -> String {
    let mut result = model.links[link].name.clone();
    while let Some(index) = model.links[link].parent_joint {
        let j = &model.joints[index];
        // Compare names and physical definitions, never vector indices. The
        // ancestor chain also detects changed fixed transforms above a joint.
        result.push_str(&format!(
            "|{}:{:?}:{:?}:{:?}:{:?}:{}:{:?}",
            j.name,
            j.joint_type,
            j.origin,
            j.axis,
            j.limits,
            model.links[j.parent_link].name,
            j.mimic
                .map(|m| (&model.joints[m.source_joint].name, m.multiplier, m.offset))
        ));
        link = j.parent_link;
    }
    result
}

fn joint_map(old: &RobotModel, new: &RobotModel) -> Vec<Option<usize>> {
    fn identities(model: &RobotModel) -> BTreeMap<String, (String, String)> {
        let mut graph = Graph::default();
        let names = graph.visit(&model.source, "robot", &mut 0);
        names
            .into_iter()
            .filter_map(|(name, (part, _))| {
                graph.parts[part]
                    .catalog()
                    .map(|(id, revision, _)| (name, (id.into(), revision.into())))
            })
            .collect()
    }
    let old_ids = identities(old);
    let new_ids = identities(new);
    new.actuated_joints
        .iter()
        .map(|&i| {
            let j = &new.joints[i];
            old.actuated_joints.iter().find_map(|&oi| {
                let oj = &old.joints[oi];
                (oj.name == j.name
                    && old_ids.get(&old.links[oj.child_link].name)
                        == new_ids.get(&new.links[j.child_link].name)
                    && joint_signature(old, oj.child_link) == joint_signature(new, j.child_link))
                .then_some(oj.q_index)
                .flatten()
            })
        })
        .collect()
}

fn migrate(q: &[f64], neutral: &[f64], map: &[Option<usize>]) -> Vec<f64> {
    map.iter()
        .zip(neutral)
        .map(|(old, default)| old.and_then(|i| q.get(i)).copied().unwrap_or(*default))
        .collect()
}

fn only_robot(scene: &Scene, index: usize) -> Scene {
    let mut alone = Scene::empty();
    alone.robots.push(scene.robots[index].clone());
    // An assembly is reusable independently of the vehicle it was riding.
    alone.robots[0].mount = None;
    let name = &alone.robots[0].name;
    alone.parts = scene
        .parts
        .iter()
        .filter(|p| {
            matches!(p.kind, PartTargetKind::Robot | PartTargetKind::Tool)
                && (p.target == *name || p.target.starts_with(&format!("{name}/")))
        })
        .cloned()
        .collect();
    alone
}

fn summary(scene: &Scene) -> Value {
    let r = &scene.robots()[0];
    let tip = r.model.default_tcp_link();
    let bom = scene.bom();
    let missing: Vec<_> = bom
        .rows
        .iter()
        .filter(|r| {
            r.attributes
                .get("mass_kg")
                .and_then(crate::part::PartAttr::as_number)
                .is_none()
        })
        .flat_map(|r| r.names.clone())
        .collect();
    json!({"tcp": {"link": r.model.links[tip].name, "pose": PoseMsg::from(&scene.link_poses_for(0)[tip])},
        "mass": {"known_kg": bom.total("mass_kg"), "missing": missing},
        "links": r.model.links.len(), "bom": bom.rows, "report": super::report(scene)})
}

/// Guidance describes evidence for the route, separately from detailed fit.
/// No product-specific rules, mesh guesses or interface-name-only approvals.
fn route(report: &MountingReport) -> &'static str {
    let interfaces: Vec<_> = report
        .items
        .iter()
        .filter(|i| i.id.ends_with(":interface"))
        .collect();
    let supported_kit = report.kits.iter().any(|k| {
        k["manufacturer_support"]["status"] == "pass" && k["composition"]["status"] == "pass"
    });
    let known = !interfaces.is_empty() && interfaces.iter().all(|i| i.status == "pass")
        || !report.simulation.connections.is_empty()
            && report
                .simulation
                .connections
                .iter()
                .all(|c| c.basis != "unknown");
    if !known && !supported_kit {
        return "needs_information";
    }
    if report
        .simulation
        .connections
        .iter()
        .any(|c| matches!(c.method, "catalog_adapter" | "custom_adapter"))
    {
        "adapter_evidence"
    } else {
        "direct_evidence"
    }
}

pub struct Preview {
    /// Isolated geometry for the viewport and reusable assembly project.
    pub candidate: Scene,
    replacement: Option<Scene>,
    pub data: Value,
}

/// Build a complete candidate Robot with attach_tool/mount, then preview it.
pub fn preview(scene: &Scene, robot: &str, model: Arc<RobotModel>) -> Result<Preview, String> {
    let index = scene
        .robot_index(robot)
        .ok_or_else(|| format!("unknown robot `{robot}`"))?;
    let old = &scene.robots[index];
    let map = joint_map(&old.model, &model);
    let q = migrate(old.joint_positions(), &model.neutral_positions(), &map);
    let (mut replacement_robot, warnings) =
        SceneRobot::new(robot.into(), model.clone(), *old.base_pose());
    replacement_robot.joint_positions = q.clone();
    let mut candidate = only_robot(scene, index);
    candidate.robots[0] = replacement_robot.clone();
    candidate.collision_warnings = warnings.clone();
    let original_pins = candidate.parts.clone();
    candidate.prune_parts();
    let unmapped_pins: Vec<_> = original_pins
        .iter()
        .filter(|p| !candidate.parts.contains(p))
        .cloned()
        .collect();
    let mut blockers: Vec<_> = unmapped_pins.iter().map(|p| format!(
        "Part annotation `{}` has no target in the candidate; it remains in the live cell and is omitted from the standalone draft. Update it before applying.", p.target
    )).collect();
    let mut project = scene.to_project();
    let candidate_project = candidate.to_project();
    let saved = &mut project.robots[index];
    saved.source = robot_source_msg(&model.source);
    saved.groups = candidate_project.robots[0].groups.clone();
    saved.joint_positions = q.clone();
    let all_old_joints = (0..old.model.dof()).all(|i| map.contains(&Some(i)));
    let affected =
        |owner: &Option<String>| owner.as_deref().unwrap_or(&scene.robots[0].name) == robot;
    for motion in &mut project.motions {
        if !affected(&motion.robot) {
            continue;
        }
        if !all_old_joints {
            blockers.push(format!(
                "Motion `{}` uses joints whose identity changed; update it before applying",
                motion.name
            ));
        }
        for segment in &mut motion.segments {
            segment.goal_positions = migrate(&segment.goal_positions, &q, &map);
        }
        project
            .mounting_revalidation
            .push(format!("motion:{}", motion.name));
    }
    for scenario in &mut project.scenarios {
        for joints in &mut scenario.joints {
            if joints.robot != robot {
                continue;
            }
            if !all_old_joints {
                blockers.push(format!(
                    "Scenario `{}` uses joints whose identity changed",
                    scenario.name
                ));
            }
            joints.positions = migrate(&joints.positions, &q, &map);
        }
    }
    // Sequences can address the arm indirectly (ramps, toolpaths, shared
    // devices, contacts). Conservatively require a fresh cell rollout.
    project.mounting_revalidation.extend(
        project
            .sequences
            .iter()
            .map(|s| format!("sequence:{}", s.name)),
    );
    project.mounting_revalidation.extend(
        project
            .toolpaths
            .iter()
            .map(|t| format!("toolpath:{}", t.name)),
    );
    project.mounting_revalidation.sort();
    project.mounting_revalidation.dedup();
    let mut replacement = Scene::empty();
    replacement.robots = scene.robots.clone();
    replacement.robots[index] = replacement_robot;
    replacement.collision_warnings = scene
        .collision_warnings
        .iter()
        .chain(&warnings)
        .cloned()
        .collect();
    // Re-resolve all index-bearing references through the existing project
    // loader, on a separate scene. A missing attachment/link/group blocks apply.
    if let Err(error) = replacement.apply_project(&project) {
        blockers.push(error.to_string());
    }
    // Keep unrelated collision exemptions. Exemptions touching changed robot
    // geometry must be regenerated, even when a link happens to keep its name.
    for (a, b) in scene.inter_acm.allowed_pairs() {
        if a.0 != index && b.0 != index {
            replacement.inter_acm.allow(a, b);
        }
    }
    let report = super::report(&candidate);
    let mounting_blockers = report.simulation_blockers();
    let can_apply =
        !report.assemblies.is_empty() && mounting_blockers.is_empty() && blockers.is_empty();
    let mounting_blockers: Vec<_> = mounting_blockers.iter().map(|item| &item.id).collect();
    let candidate_revision = revision(&candidate);
    let before = only_robot(scene, index);
    let data = json!({"schema_version":"1", "robot":robot, "base_revision":revision(scene),
        "candidate_revision":candidate_revision, "source":robot_source_msg(&model.source),
        "project":candidate_project, "route":route(&report), "can_apply":can_apply,
        "mounting_blockers":mounting_blockers,
        "blockers":blockers, "unmapped_pins":unmapped_pins, "revalidation":project.mounting_revalidation,
        "preserved_joints": model.actuated_joints.iter().zip(&map).filter(|(_,q)| q.is_some()).map(|(&j,_)| &model.joints[j].name).collect::<Vec<_>>(),
        "reset_joints": model.actuated_joints.iter().zip(&map).filter(|(_,q)| q.is_none()).map(|(&j,_)| &model.joints[j].name).collect::<Vec<_>>(),
        "before":summary(&before), "after":summary(&candidate), "before_inspection":inspection(&before), "inspection":inspection(&candidate)});
    Ok(Preview {
        candidate,
        replacement: blockers.is_empty().then_some(replacement),
        data,
    })
}

/// Rebuild, re-inspect and compare both snapshots before the atomic swap.
pub fn apply(
    scene: &mut Scene,
    robot: &str,
    model: Arc<RobotModel>,
    base_revision: &str,
    candidate_revision: &str,
) -> Result<Value, String> {
    if revision(scene) != base_revision {
        return Err("Scene changed after preview. Preview the candidate again.".into());
    }
    let preview = preview(scene, robot, model)?;
    if preview.data["candidate_revision"] != candidate_revision {
        return Err("Candidate inputs changed after preview. Preview again.".into());
    }
    if preview.data["can_apply"] != true {
        return Err("Mounting or scene references remain unresolved. Save this proposal as a draft or resolve the findings before applying.".into());
    }
    let generation = scene.assembly_generation.wrapping_add(1);
    *scene = preview
        .replacement
        .ok_or("Candidate cannot replace this robot")?;
    scene.assembly_generation = generation;
    Ok(preview.data)
}

/// Source paths select real Composite nodes, including nested assemblies.
pub fn connections(source: &Value, path: &str) -> Vec<Value> {
    let mut result = Vec::new();
    if source["kind"] == "composite" && source["role"].as_str().unwrap_or("tool") == "tool" {
        result.push(json!({"path":path,"flange":source["flange"],"mount":source["mount"]}));
    }
    for field in ["base", "tool", "inner"] {
        if source.get(field).is_some() {
            result.extend(connections(&source[field], &format!("{path}/{field}")));
        }
    }
    result
}

/// Reusable project file for one assembly, preserving catalog/kit provenance.
pub fn assembly_project(scene: &Scene, robot: &str) -> Result<ProjectFile, String> {
    let index = scene
        .robot_index(robot)
        .ok_or_else(|| format!("unknown robot `{robot}`"))?;
    Ok(only_robot(scene, index).to_project())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changed_mimic_ancestor_does_not_transfer_descendant_pose() {
        let xml = r#"<robot name="test"><link name="root"/><link name="drive"/><link name="carrier"/><link name="tip"/>
        <joint name="driver" type="continuous"><parent link="root"/><child link="drive"/></joint>
        <joint name="follower" type="continuous"><parent link="root"/><child link="carrier"/><mimic joint="driver" multiplier="1"/></joint>
        <joint name="axis" type="continuous"><parent link="carrier"/><child link="tip"/></joint></robot>"#;
        let old = RobotModel::from_urdf_str(xml).unwrap();
        let new = RobotModel::from_urdf_str(&xml.replace("multiplier=\"1\"", "multiplier=\"2\""))
            .unwrap();
        let map: BTreeMap<_, _> = new
            .actuated_joint_names()
            .into_iter()
            .zip(joint_map(&old, &new))
            .collect();
        assert!(map["driver"].is_some());
        assert_eq!(map["axis"], None);
    }
}
