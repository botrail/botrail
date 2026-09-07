//! Compare voxel and source-surface hulls without changing the production path.
//! Input: {urdf, q, stages: {name: {link: mesh_path}}, pairs: [[link, link]]}.
//! Output includes link-local meshes and world poses for a browser overlay.
use std::{collections::BTreeMap, path::Path, time::Instant};

use botrail_collide::{mesh::compound_from_hulls, to_parry_pose};
use parry3d_f64::{
    math::Vector,
    query,
    transformation::vhacd::{VHACDParameters, VHACD},
};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize)]
struct Input {
    urdf: String,
    q: Vec<f64>,
    stages: BTreeMap<String, BTreeMap<String, String>>,
    pairs: Vec<[String; 2]>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let input: Input = serde_json::from_slice(&std::fs::read(&args[1])?)?;
    let model = botrail_model::RobotModel::from_urdf_file(&input.urdf)?;
    let fk = botrail_kin::forward_kinematics(&model, &input.q)?;
    let poses: BTreeMap<_, _> = model
        .links
        .iter()
        .zip(&fk)
        .map(|(link, pose)| (link.name.clone(), to_parry_pose(pose)))
        .collect();
    let mut output = serde_json::Map::new();
    for (stage, files) in &input.stages {
        let mut shapes = BTreeMap::new();
        let mut meshes = serde_json::Map::new();
        for (link, path) in files {
            let mesh = botrail_mesh::load_path(Path::new(path))?;
            let points: Vec<_> = mesh
                .vertices
                .iter()
                .map(|p| Vector::from_array(*p))
                .collect();
            let start = Instant::now();
            let decomposition = VHACD::decompose(
                &VHACDParameters {
                    resolution: 64,
                    ..Default::default()
                },
                &points,
                &mesh.indices,
                true,
            );
            let decompose_ms = start.elapsed().as_secs_f64() * 1000.0;
            let mut variants = serde_json::Map::new();
            for kind in ["voxel", "surface"] {
                let start = Instant::now();
                let hulls = if kind == "voxel" {
                    decomposition.compute_convex_hulls(0)
                } else {
                    decomposition.compute_exact_convex_hulls(&points, &mesh.indices)
                };
                let hull_points: Vec<Vec<[f64; 3]>> = hulls
                    .iter()
                    .map(|(p, _)| p.iter().map(|v| v.to_array()).collect())
                    .collect();
                let shape = compound_from_hulls(&hull_points)?;
                let build_ms = start.elapsed().as_secs_f64() * 1000.0;
                let aabb = shape.compute_local_aabb();
                let geometry: Vec<_> = hulls.iter().map(|(p, i)| json!({
                    "vertices": p.iter().map(|v| v.to_array()).collect::<Vec<_>>(), "indices": i,
                })).collect();
                variants.insert(
                    kind.into(),
                    json!({"hulls":geometry,"build_ms":build_ms,
                    "aabb":[aabb.mins.to_array(),aabb.maxs.to_array()]}),
                );
                shapes.insert((link.clone(), kind), shape);
            }
            let pose = poses[link];
            meshes.insert(
                link.clone(),
                json!({"vertices":mesh.vertices,"indices":mesh.indices,
                "position":pose.translation.to_array(),"quaternion":pose.rotation.to_array(),
                "decompose_ms":decompose_ms,"variants":variants}),
            );
            eprintln!("{stage}/{link}: {decompose_ms:.0} ms");
        }
        let mut pairs: Vec<Value> = Vec::new();
        for [a, b] in &input.pairs {
            for kind in ["voxel", "surface"] {
                let sa = &shapes[&(a.clone(), kind)];
                let sb = &shapes[&(b.clone(), kind)];
                pairs.push(json!({"pair":[a,b],"kind":kind,
                    "intersects":query::intersection_test(&poses[a],sa.as_ref(),&poses[b],sb.as_ref())?,
                    "distance_mm":query::distance(&poses[a],sa.as_ref(),&poses[b],sb.as_ref())?*1000.0}));
            }
        }
        output.insert(stage.clone(), json!({"meshes":meshes,"pairs":pairs}));
    }
    std::fs::write(&args[2], serde_json::to_vec(&output)?)?;
    Ok(())
}
