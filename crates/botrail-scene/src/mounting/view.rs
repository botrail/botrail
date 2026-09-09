//! Read-only rendering references accompanying the shared mounting report.
//! Poses are a snapshot; presentation offsets never enter the Scene or checker.

use super::*;

/// Build one report and its display references from the same scene snapshot.
/// The versioned presentation envelope leaves the Python report contract intact.
pub fn inspection(scene: &Scene) -> Value {
    let mut graph = Graph::default();
    let mut connections = Vec::new();
    let mut robots = Vec::new();
    for (ri, robot) in scene.robots().iter().enumerate() {
        let first_part = graph.parts.len();
        let first_edge = graph.edges.len();
        let names = graph.visit(&robot.model.source, &robot.name, &mut 0);
        let model = &robot.model;
        let poses = scene.link_poses_for(ri);
        // Source names retain exact attachment prefixes. For USD links omitted
        // from the declarations, propagate ownership through the assembled tree,
        // stopping at every explicitly owned mount root. No basename guessing.
        let mut owners = vec![None; model.links.len()];
        for (name, (owner, _)) in &names {
            if let Some(index) = model.link_index(name) {
                owners[index] = Some(*owner);
            }
        }
        owners[model.root_link].get_or_insert(first_part);
        for &ji in &model.joint_order {
            let joint = &model.joints[ji];
            if owners[joint.child_link].is_none() {
                owners[joint.child_link] = owners[joint.parent_link];
            }
        }
        let frame_index = |owner: Option<usize>, frame: &str| -> Option<usize> {
            let owner = owner?;
            let mut matches = names.iter().filter_map(|(name, (part, original))| {
                (*part == owner && original == frame)
                    .then(|| model.link_index(name))
                    .flatten()
            });
            let index = matches.next()?;
            matches.next().is_none().then_some(index)
        };
        let face_view = |owner: Option<usize>, frame: &str, role: InterfaceRole| {
            let part = owner.map(|i| &graph.parts[i]);
            let face = part.and_then(|p| p.face(frame, role));
            let index = frame_index(owner, frame);
            json!({
                "part": part.map(|p| &p.name), "frame": frame,
                "link": index.map(|i| &model.links[i].name),
                "pose": index.map(|i| PoseMsg::from(&poses[i])),
                "declaration": face,
                "geometry_mapped": index.is_some() && face.and_then(|f| f.geometry.as_ref())
                    .zip(part).is_some_and(|(g,p)| g.frame_verified && p.supported(&g.evidence)),
                "clearance_mapped": index.is_some() && face.and_then(|f| f.clearance.as_ref())
                    .zip(part).is_some_and(|(g,p)| g.frame_verified && p.supported(&g.evidence)),
                "sources": part.map(|p| &p.meta.sources),
            })
        };
        for edge in &graph.edges[first_edge..] {
            if edge.role != MountRole::Tool {
                continue;
            }
            let flange = frame_index(edge.base, &edge.flange);
            let mount = frame_index(edge.tool, &edge.mount);
            let descendants = mount.map(|root| {
                let mut links = vec![false; model.links.len()];
                links[root] = true;
                for &ji in &model.joint_order {
                    let j = &model.joints[ji];
                    links[j.child_link] |= links[j.parent_link];
                }
                model
                    .links
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| links[*i])
                    .map(|(_, link)| link.name.clone())
                    .collect::<Vec<_>>()
            });
            let arm = model
                .groups()
                .into_iter()
                .find(|g| {
                    flange.is_some_and(|mut link| loop {
                        if Some(link) == g.flange {
                            break true;
                        }
                        let Some(ji) = model.links[link].parent_joint else {
                            break false;
                        };
                        link = model.joints[ji].parent_link;
                    })
                })
                .map(|g| g.name);
            connections.push(json!({
                "target": edge.target, "robot": robot.name, "arm": arm,
                "flange": face_view(edge.base, &edge.flange, InterfaceRole::Flange),
                "mount": face_view(edge.tool, &edge.mount, InterfaceRole::Mount),
                "offset": edge.offset, "moving_links": descendants.unwrap_or_default(),
            }));
        }
        robots.push(json!({
            "name": robot.name,
            "edit_connections": super::edit::connections(&serde_json::to_value(crate::project::robot_source_msg(&model.source)).unwrap(), ""),
            "links": model.links.iter().enumerate().map(|(i, link)| json!({
                "name": link.name, "part": owners[i].map(|p| &graph.parts[p].name),
                "pose": PoseMsg::from(&poses[i]),
            })).collect::<Vec<_>>(),
        }));
    }
    let parts: Vec<_> = graph
        .parts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            json!({
                "id": p.name, "name": p.meta.product.as_deref().unwrap_or(&p.name),
                "catalog": p.catalog().map(|(id, _, _)| id),
                "kit": graph.kits.iter().find(|k| k.members.contains(&i)).map(|k| &k.name),
            })
        })
        .collect();
    let report = evaluate(
        graph,
        scene.parts(),
        &scene.connection_plan().configurations,
    );
    json!({"schema_version": "1", "report": report, "connections": connections,
        "parts": parts, "robots": robots, "revalidation": scene.mounting_revalidation})
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
    use std::sync::Arc;

    #[test]
    fn prefixed_nested_parts_resolve_to_actual_links_without_changing_report() {
        let base = RobotModel::from_urdf_str(r#"<robot name="arm"><link name="root"/><link name="face"/>
          <joint name="j" type="fixed"><parent link="root"/><child link="face"/><origin xyz="0 0 0.4"/></joint></robot>"#).unwrap();
        let mut tool = RobotModel::from_urdf_str(r#"<robot name="tool"><link name="root"/><link name="face"/>
          <joint name="j" type="fixed"><parent link="root"/><child link="face"/><origin xyz="0 0 0.02"/></joint></robot>"#).unwrap();
        tool.flange_link = tool.link_index("face");
        let assembled = base
            .attach_tool(
                &tool,
                Some("face"),
                Some("root"),
                Isometry3::identity(),
                None,
                Some("a_"),
                None,
            )
            .unwrap()
            .attach_tool(
                &tool,
                Some("a_face"),
                Some("root"),
                Isometry3::identity(),
                None,
                Some("b_"),
                None,
            )
            .unwrap();
        let scene = Scene::new(Arc::new(assembled));
        let before = serde_json::to_value(report(&scene)).unwrap();
        let view = inspection(&scene);
        assert_eq!(view["report"], before);
        assert_eq!(view["connections"][0]["mount"]["link"], "a_root");
        assert_eq!(view["connections"][1]["flange"]["link"], "a_face");
        assert_eq!(view["connections"][1]["mount"]["link"], "b_root");
        assert_eq!(
            view["connections"][0]["moving_links"],
            json!(["a_root", "a_face", "b_root", "b_face"])
        );
        assert_eq!(
            view["connections"][0]["flange"]["pose"]["position"],
            json!([0., 0., 0.4])
        );
        assert_eq!(view["connections"][0]["flange"]["geometry_mapped"], false);
        assert_eq!(serde_json::to_value(report(&scene)).unwrap(), before);
    }

    #[test]
    fn empty_scene_and_undeclared_tool_keep_the_original_report() {
        let empty = Scene::empty();
        assert_eq!(inspection(&empty)["connections"], json!([]));
        assert_eq!(
            inspection(&empty)["report"],
            serde_json::to_value(report(&empty)).unwrap()
        );
        let model = RobotModel::from_urdf_str(r#"<robot name="plain"><link name="root"/></robot>"#)
            .unwrap();
        let scene = Scene::new(Arc::new(model));
        let view = inspection(&scene);
        assert_eq!(view["connections"], json!([]));
        assert_eq!(view["robots"][0]["links"][0]["part"], "plain");
    }

    #[test]
    fn tool_subtree_and_identical_instances_keep_separate_references() {
        let base =
            RobotModel::from_urdf_str(r#"<robot name="arm"><link name="root"/></robot>"#).unwrap();
        let tool = RobotModel::from_urdf_str(r#"<robot name="tool"><link name="body"/><link name="mount"/><link name="sibling"/>
          <joint name="a" type="fixed"><parent link="body"/><child link="mount"/><origin xyz="0 0 0.02"/></joint>
          <joint name="b" type="fixed"><parent link="body"/><child link="sibling"/></joint></robot>"#).unwrap();
        let model = Arc::new(
            base.attach_tool(
                &tool,
                Some("root"),
                Some("body"),
                Isometry3::identity(),
                None,
                Some("t_"),
                None,
            )
            .unwrap(),
        );
        let mut scene = Scene::new(model.clone());
        scene.add_robot(model, Some("other"), Isometry3::translation(1., 0., 0.));
        let view = inspection(&scene);
        let edges = view["connections"].as_array().unwrap();
        assert_eq!(edges.len(), 2);
        assert_ne!(edges[0]["target"], edges[1]["target"]);
        assert_eq!(
            edges[0]["moving_links"],
            json!(["t_body", "t_mount", "t_sibling"])
        );
        assert_eq!(edges[0]["mount"]["link"], "t_body");
        assert_eq!(edges[1]["mount"]["pose"]["position"], json!([1., 0., 0.]));
    }
}
