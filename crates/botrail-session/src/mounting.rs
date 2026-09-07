//! Shared assembly editor transport. All decisions come from botrail-scene.
use super::*;
use botrail_model::RobotModel;
use botrail_scene::{
    mounting::edit as core,
    project::{self, RobotSourceMsg},
};
use serde_json::{json, Value};
use std::sync::Arc;

pub fn load_embedded(
    input: &Value,
    import_usd: &dyn Fn(&str, &str) -> Result<RobotModel, String>,
) -> Result<RobotModel, String> {
    let mut project_robot = None;
    let source: RobotSourceMsg = match input["kind"].as_str() {
        Some("source") => serde_json::from_value(input["source"].clone()).map_err(|e| format!("Robot source: {e}"))?,
        Some("project") => {
            let file = project::ProjectFile::from_json(&input["project"].to_string()).map_err(|e| e.to_string())?;
            let r = if let Some(name) = input["robot"].as_str() {
                file.robots.iter().find(|r| r.name.as_deref() == Some(name)).ok_or("Selected project robot does not exist")?
            } else if file.robots.len() == 1 { &file.robots[0] }
            else { return Err("Select one robot from the saved project".into()); };
            project_robot = Some(r.clone());
            r.source.clone()
        }
        _ => return Err("Catalog/package loading requires the Python Studio server. Load a saved URDF assembly project in this session.".into()),
    };
    let mut model = project::model_from_source(&source, import_usd).map_err(|e| e.to_string())?;
    if let Some(r) = project_robot {
        project::apply_declared_groups(&mut model, &r).map_err(|e| e.to_string())?;
    }
    Ok(model)
}

fn text<'a>(data: &'a Value, key: &str) -> Result<&'a str, String> {
    data[key]
        .as_str()
        .filter(|s| !s.is_empty())
        .ok_or_else(|| format!("Missing {key}"))
}

fn candidate(host: &impl SessionHost, scene: &Scene, data: &Value) -> Result<RobotModel, String> {
    let robot = text(data, "robot")?;
    let index = scene.robot_index(robot).ok_or("Robot no longer exists")?;
    let base = &scene.robots()[index].model;
    let part = host.load_mounting_model(&data["input"])?;
    let optional = |key: &str| data[key].as_str().filter(|s| !s.is_empty());
    match data["operation"].as_str().unwrap_or("replace") {
        "replace" => Ok(part),
        "attach" => base
            .attach_tool(
                &part,
                optional("flange"),
                optional("mount"),
                Isometry3::identity(),
                optional("tcp"),
                optional("prefix"),
                optional("group"),
            )
            .map_err(|e| e.to_string()),
        "insert_adapter" => {
            let mut source = serde_json::to_value(project::robot_source_msg(&base.source)).unwrap();
            let path = data["connection_path"]
                .as_str()
                .ok_or("Select a tool connection")?;
            let edge = source
                .pointer_mut(path)
                .ok_or("Tool connection no longer exists")?;
            if edge["kind"] != "composite" || edge["role"].as_str().unwrap_or("tool") != "tool" {
                return Err("Select a tool attachment, not an arm mount".into());
            }
            let prefix = optional("prefix").unwrap_or("adapter_");
            let mount = optional("mount")
                .map(str::to_string)
                .or_else(|| part.mount_link.map(|i| part.links[i].name.clone()))
                .unwrap_or_else(|| part.links[part.root_link].name.clone());
            let flange = optional("flange")
                .map(str::to_string)
                .or_else(|| part.flange_link.map(|i| part.links[i].name.clone()))
                .ok_or("The adapter has no declared output flange. Specify its flange link.")?;
            if part.link_index(&flange).is_none() {
                return Err(format!("Unknown adapter flange `{flange}`"));
            }
            edge["base"] = json!({"kind":"composite", "base":edge["base"], "tool":project::robot_source_msg(&part.source),
                "flange":edge["flange"], "mount":mount, "offset":PoseMsg::from(&Isometry3::identity()), "prefix":prefix, "group":edge["group"]});
            edge["flange"] = json!(format!("{prefix}{flange}"));
            host.load_mounting_model(&json!({"kind":"source", "source":source}))
        }
        other => Err(format!("Unknown assembly operation `{other}`")),
    }
}

pub fn edit(host: &impl SessionHost, action: &str, data: &Value) -> Result<Value, String> {
    let robot = text(data, "robot")?;
    match action {
        "save" => host
            .with_scene(|scene| core::assembly_project(scene, robot))
            .map(|project| json!({"project":project})),
        "preview" => {
            let snapshot = host.snapshot();
            let model = Arc::new(candidate(host, &snapshot, data)?);
            let mut preview = core::preview(&snapshot, robot, model)?;
            // Never use a live-robot USD URL for a different candidate model.
            // Link visuals receive their own immutable mesh registrations.
            preview.data["scene"] = serde_json::to_value(SceneDescriptionMsg::from_scene(
                &preview.candidate,
                |p| host.mesh_url(p),
                |_| None,
            ))
            .unwrap();
            preview.data["before_scene"] = serde_json::to_value(SceneDescriptionMsg::from_scene(
                &snapshot,
                |p| host.mesh_url(p),
                |_| None,
            ))
            .unwrap();
            Ok(preview.data)
        }
        "apply" => {
            // The source in the preview is pinned and embedded. Never resolve a
            // floating catalog query a second time at application time.
            let model = Arc::new(
                host.load_mounting_model(&json!({"kind":"project", "project":data["project"]}))?,
            );
            let result = host.with_scene(|scene| {
                core::apply(
                    scene,
                    robot,
                    model,
                    text(data, "base_revision")?,
                    text(data, "candidate_revision")?,
                )
            })?;
            host.invalidate_mounting_results();
            for message in initial_messages(host) {
                host.emit(&message);
            }
            Ok(json!({"applied":true, "revalidation":result["revalidation"]}))
        }
        _ => Err("Unknown assembly editor action".into()),
    }
}
