//! Mesh collision shapes: VHACD convex decomposition into a solid compound
//! (per the shape policy — raw TriMesh misses containment), with a
//! content-addressed disk cache so the ~1s/mesh decomposition cost is paid
//! once per distinct (file, scale) pair.

use std::hash::{Hash, Hasher};
use std::path::PathBuf;

use botrail_mesh::MeshData;
use nalgebra::Vector3;
use parry3d_f64::math::{Pose, Vector};
use parry3d_f64::shape::SharedShape;
use parry3d_f64::transformation::vhacd::{VHACDParameters, VHACD};

use crate::CollideError;

/// Voxelization resolution for VHACD (bench-validated starting point, see
/// docs/bench-parry3d.md: ~0.8-1.0s/mesh, 12-17 pieces at 64).
pub const VHACD_RESOLUTION: u32 = 64;

/// Bump when the decomposition parameters or cache layout change; stale
/// entries then miss instead of deserializing garbage.
const CACHE_VERSION: u32 = 1;

/// Decomposes a triangle mesh into a compound of convex hulls (VHACD).
/// Pure and filesystem-free — the entry point for mesh data that arrives
/// from memory (importers, wasm).
pub fn mesh_to_compound(mesh: &MeshData) -> Result<SharedShape, CollideError> {
    compound_from_point_sets(&decompose(mesh))
}

/// Loads a mesh file (STL/OBJ), applies `scale`, and returns its VHACD
/// compound, consulting the disk cache first.
pub fn load_mesh_compound(
    path: &std::path::Path,
    scale: &Vector3<f64>,
) -> Result<SharedShape, CollideError> {
    let bytes = std::fs::read(path)
        .map_err(|e| CollideError::MeshLoad(format!("{}: {e}", path.display())))?;
    let key = cache_key(&bytes, scale);
    if let Some(shape) = read_cache(&key) {
        return Ok(shape);
    }

    let ext = path
        .extension()
        .map(|e| e.to_string_lossy().to_lowercase())
        .unwrap_or_default();
    let mesh = match ext.as_str() {
        "stl" => botrail_mesh::parse_stl(&bytes),
        "obj" => botrail_mesh::parse_obj(&String::from_utf8_lossy(&bytes)),
        other => {
            return Err(CollideError::MeshLoad(format!(
                "{}: unsupported mesh format `{other}` (stl/obj)",
                path.display()
            )))
        }
    }
    .map_err(|e| CollideError::MeshLoad(format!("{}: {e}", path.display())))?;

    let hulls = decompose(&mesh.scaled([scale.x, scale.y, scale.z]));
    let shape = compound_from_point_sets(&hulls)?;
    write_cache(&key, &hulls);
    Ok(shape)
}

/// VHACD hulls as plain point sets — the cacheable representation.
fn decompose(mesh: &MeshData) -> Vec<Vec<[f64; 3]>> {
    let points: Vec<Vector> = mesh
        .vertices
        .iter()
        .map(|v| Vector::new(v[0], v[1], v[2]))
        .collect();
    let params = VHACDParameters {
        resolution: VHACD_RESOLUTION,
        ..Default::default()
    };
    let vhacd = VHACD::decompose(&params, &points, &mesh.indices, true);
    vhacd
        .compute_convex_hulls(0)
        .into_iter()
        .map(|(pts, _)| pts.iter().map(|p| [p.x, p.y, p.z]).collect())
        .collect()
}

fn compound_from_point_sets(hulls: &[Vec<[f64; 3]>]) -> Result<SharedShape, CollideError> {
    let shapes: Vec<(Pose, SharedShape)> = hulls
        .iter()
        .filter_map(|pts| {
            let points: Vec<Vector> = pts.iter().map(|p| Vector::new(p[0], p[1], p[2])).collect();
            SharedShape::convex_hull(&points)
        })
        .map(|s| (Pose::identity(), s))
        .collect();
    if shapes.is_empty() {
        return Err(CollideError::MeshLoad(
            "convex decomposition produced no parts (degenerate mesh?)".to_string(),
        ));
    }
    Ok(SharedShape::compound(shapes))
}

// ----------------------------------------------------------------- cache

/// `BOTRAIL_CACHE_DIR` override, else `~/.cache/botrail`, else the system
/// temp dir. Subdirectory `vhacd/` holds one JSON file per key.
fn cache_dir() -> PathBuf {
    let base = std::env::var_os("BOTRAIL_CACHE_DIR")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".cache").join("botrail"))
        })
        .unwrap_or_else(|| std::env::temp_dir().join("botrail-cache"));
    base.join("vhacd")
}

fn cache_key(bytes: &[u8], scale: &Vector3<f64>) -> String {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    CACHE_VERSION.hash(&mut hasher);
    VHACD_RESOLUTION.hash(&mut hasher);
    bytes.hash(&mut hasher);
    for s in [scale.x, scale.y, scale.z] {
        s.to_bits().hash(&mut hasher);
    }
    format!("{:x}-{:016x}.json", bytes.len(), hasher.finish())
}

fn read_cache(key: &str) -> Option<SharedShape> {
    let bytes = std::fs::read(cache_dir().join(key)).ok()?;
    let hulls: Vec<Vec<[f64; 3]>> = serde_json::from_slice(&bytes).ok()?;
    compound_from_point_sets(&hulls).ok()
}

/// Best-effort: a failed cache write only costs the next load a re-run.
fn write_cache(key: &str, hulls: &[Vec<[f64; 3]>]) {
    let dir = cache_dir();
    if std::fs::create_dir_all(&dir).is_err() {
        return;
    }
    if let Ok(json) = serde_json::to_vec(hulls) {
        let _ = std::fs::write(dir.join(key), json);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use parry3d_f64::query;

    #[test]
    fn box_mesh_decomposes_to_solid_compound() {
        let shape = mesh_to_compound(&botrail_mesh::box_mesh([0.4, 0.4, 0.4])).unwrap();
        // A ball centered inside the box: solid semantics must report
        // intersection (raw TriMesh would miss this containment).
        let ball = SharedShape::ball(0.05);
        let hit = query::intersection_test(
            &Pose::identity(),
            shape.as_ref(),
            &Pose::identity(),
            ball.as_ref(),
        )
        .unwrap();
        assert!(hit, "ball inside mesh box must collide");

        // Ball clearly outside: no intersection, sane positive distance.
        let outside = Pose {
            translation: Vector::new(1.0, 0.0, 0.0),
            ..Pose::identity()
        };
        let hit = query::intersection_test(&Pose::identity(), shape.as_ref(), &outside, ball.as_ref())
            .unwrap();
        assert!(!hit);
        let d = query::distance(&Pose::identity(), shape.as_ref(), &outside, ball.as_ref()).unwrap();
        assert!((d - 0.75).abs() < 0.02, "distance {d}");
    }

    #[test]
    fn mesh_containing_mesh_collides() {
        let big = mesh_to_compound(&botrail_mesh::box_mesh([1.0, 1.0, 1.0])).unwrap();
        let small = mesh_to_compound(&botrail_mesh::box_mesh([0.1, 0.1, 0.1])).unwrap();
        let hit = query::intersection_test(
            &Pose::identity(),
            big.as_ref(),
            &Pose::identity(),
            small.as_ref(),
        )
        .unwrap();
        assert!(hit, "fully contained mesh must collide (solid policy)");
    }

    #[test]
    fn file_load_uses_cache() {
        let dir = std::env::temp_dir().join(format!("botrail-mesh-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // Point the cache at the scratch dir for this test.
        std::env::set_var("BOTRAIL_CACHE_DIR", &dir);

        let stl = dir.join("box.stl");
        std::fs::write(&stl, botrail_mesh::to_stl_binary(&botrail_mesh::box_mesh([0.2, 0.2, 0.2])))
            .unwrap();

        let scale = Vector3::new(1.0, 1.0, 2.0);
        let first = load_mesh_compound(&stl, &scale).unwrap();
        let cached: Vec<_> = std::fs::read_dir(dir.join("vhacd")).unwrap().collect();
        assert_eq!(cached.len(), 1, "one cache entry after first load");
        let second = load_mesh_compound(&stl, &scale).unwrap();

        // Same compound either way; scale respected (z half-extent = 0.2).
        // VHACD voxelization inflates hulls slightly, hence the loose tol.
        for shape in [&first, &second] {
            let aabb = shape.compute_local_aabb();
            assert!((aabb.maxs.z - 0.2).abs() < 0.02, "z max {}", aabb.maxs.z);
            assert!((aabb.maxs.x - 0.1).abs() < 0.02, "x max {}", aabb.maxs.x);
        }

        std::env::remove_var("BOTRAIL_CACHE_DIR");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
