//! USD scene importer: composes a stage (usda/usdc/usdz, references,
//! variants, instancing) via openusd and extracts the *static environment*
//! as botrail geometry — world-posed primitives and meshes, normalized to
//! botrail conventions (meters, Z-up).
//!
//! Boundary rules (design/DESIGN.md):
//! - The core never sees USD: the importer emits [`ImportedScene`] built
//!   from `botrail_model::Geometry` + nalgebra poses.
//! - Extracted meshes are materialized as content-hashed binary STL files
//!   in the cache, so the existing mesh pipeline (VHACD collision cache,
//!   `/meshes` serving, studio display) applies unchanged.
//! - USD articulation subtrees (robots) are NOT imported as geometry —
//!   they would otherwise become static obstacles the robot self-collides
//!   with. Their root prim paths are reported via
//!   [`ImportedScene::robot_roots`]; import them with
//!   [`import_robot`].
//!
//! Filtering: only visible, default-purpose prims are imported (guide /
//! proxy / render are skipped). Leaf `Xform`/`Scope` prims with no children
//! become named [`ImportedFrame`]s — mount-point markers for robot
//! placement.

mod articulation;
pub mod export;
pub mod recording;

pub use articulation::{import_robot, import_robot_bundle, ImportedRobot, RobotImportOptions};

use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::io;
use std::path::{Path, PathBuf};

use botrail_mesh::MeshData;
use botrail_model::Geometry;
use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};
use openusd::ar::{Asset, DefaultResolver, ResolvedPath, Resolver};
use openusd::schemas::geom::{
    self, Boundable, Gprim, Imageable, PointBased, Purpose, Visibility, Xformable,
};
use openusd::usd::{Prim, SchemaBase, SchemaKind, Stage, TimeCode};
use openusd::{gf, sdf};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum UsdImportError {
    #[error("failed to open USD stage `{path}`: {message}")]
    Open { path: String, message: String },
    #[error("failed to traverse USD stage: {0}")]
    Traverse(String),
    #[error("articulation import failed: {0}")]
    Articulation(String),
    #[error("failed to write extracted mesh: {0}")]
    MeshCache(#[from] io::Error),
    #[error("recording import failed: {0}")]
    Recording(String),
}

#[derive(Debug, Clone, Default)]
pub struct ImportOptions {
    /// Roots that asset references resolve against. `omniverse://host/...`
    /// and `http(s)://host/...` references are stripped to their path and
    /// searched here too (point this at a locally downloaded asset pack).
    pub search_paths: Vec<PathBuf>,
    /// Where extracted meshes are written; defaults to
    /// `$BOTRAIL_CACHE_DIR|~/.cache/botrail|$TMP/botrail-cache` + `/usd-meshes`.
    pub mesh_cache_dir: Option<PathBuf>,
    /// Keep extracted meshes in memory ([`ImportedNode::mesh_data`])
    /// instead of writing STL files — required on wasm, where there is no
    /// filesystem. The node's `Geometry::Mesh` path is then a virtual
    /// `usd://<prim>` identifier.
    pub meshes_in_memory: bool,
}

/// One geometry prim, ready to become a scene obstacle.
#[derive(Debug, Clone)]
pub struct ImportedNode {
    /// Prim path (the naming contract shared with client-side USD loaders).
    pub name: String,
    pub geometry: Geometry,
    /// World pose, meters / Z-up.
    pub pose: Isometry3<f64>,
    /// Authored `primvars:displayColor`, linear RGB. `None` when the prim
    /// leaves it to the renderer.
    pub color: Option<[f32; 3]>,
    /// Present when [`ImportOptions::meshes_in_memory`] is set: the baked
    /// (normalized) triangle mesh, for direct collider construction.
    pub mesh_data: Option<MeshData>,
}

/// A named mount point (leaf Xform/Scope with no geometry).
#[derive(Debug, Clone)]
pub struct ImportedFrame {
    pub name: String,
    pub pose: Isometry3<f64>,
}

#[derive(Debug)]
pub struct ImportedScene {
    pub nodes: Vec<ImportedNode>,
    pub frames: Vec<ImportedFrame>,
    /// Prims that could not be imported (unsupported types, degenerate
    /// data). Import continues past them.
    pub warnings: Vec<String>,
    /// The stage's authored up axis (`"Y"` or `"Z"`). Everything in
    /// `nodes`/`frames` is already normalized to Z-up; clients rendering
    /// the *original* stage themselves need this to match.
    pub up_axis: &'static str,
    /// Root prim paths of articulations found (and skipped) in the stage —
    /// the discovery handle for importing each robot via [`import_robot`]
    /// with `articulation_root`.
    pub robot_roots: Vec<String>,
}

impl Default for ImportedScene {
    fn default() -> Self {
        ImportedScene {
            nodes: Vec::new(),
            frames: Vec::new(),
            warnings: Vec::new(),
            up_axis: "Y",
            robot_roots: Vec::new(),
        }
    }
}

/// Untyped view over any prim so `Xformable`'s provided methods (the full
/// xformOpOrder evaluator) work during a generic traversal.
pub(crate) struct AnyPrim(pub(crate) Prim);

impl SchemaBase for AnyPrim {
    const KIND: SchemaKind = SchemaKind::ConcreteTyped;
    fn prim(&self) -> &Prim {
        &self.0
    }
}
impl Imageable for AnyPrim {}
impl Xformable for AnyPrim {}
// Gprim only adds attribute accessors, so an untyped view can read the
// display primvars off any prim without knowing its concrete schema.
impl Boundable for AnyPrim {}
impl Gprim for AnyPrim {}

pub fn import_usd(path: &Path, options: &ImportOptions) -> Result<ImportedScene, UsdImportError> {
    let mut search_paths = options.search_paths.clone();
    if let Some(dir) = path.parent() {
        search_paths.push(dir.to_path_buf());
    }
    let stage = Stage::builder()
        .resolver(SearchPathResolver::new(search_paths))
        .open(&path.display().to_string())
        .map_err(|e| UsdImportError::Open {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;
    import_stage(&stage, options)
}

/// Imports a stage held entirely in memory (single layer or usdz package —
/// the wasm / drag-and-drop path; external references cannot resolve).
pub fn import_usd_bytes(
    bytes: Vec<u8>,
    file_name: &str,
    options: &ImportOptions,
) -> Result<ImportedScene, UsdImportError> {
    let stage = Stage::builder()
        .resolver(bytes_resolver(bytes, file_name))
        .open(file_name)
        .map_err(|e| UsdImportError::Open {
            path: file_name.to_string(),
            message: e.to_string(),
        })?;
    import_stage(&stage, options)
}

fn import_stage(stage: &Stage, options: &ImportOptions) -> Result<ImportedScene, UsdImportError> {
    let mut importer = Importer {
        stage,
        // USD defaults: centimeters, Y-up.
        meters_per_unit: 0.01,
        up_axis_fix: y_up_to_z_up(),
        mesh_cache_dir: options.mesh_cache_dir.clone(),
        meshes_in_memory: options.meshes_in_memory,
        out: ImportedScene::default(),
    };
    importer.read_stage_metadata();
    importer.out.up_axis = if importer.up_axis_fix == UnitQuaternion::identity() {
        "Z"
    } else {
        "Y"
    };

    let root = stage.prim(sdf::Path::abs_root());
    importer
        .walk(root, gf::Matrix4d::default(), None)
        .map_err(|e| UsdImportError::Traverse(e.to_string()))?;
    Ok(importer.out)
}

/// Single-asset in-memory resolver for [`import_usd_bytes`].
pub(crate) fn bytes_resolver(bytes: Vec<u8>, file_name: &str) -> MemoryResolver {
    MemoryResolver {
        name: file_name.to_string(),
        bytes,
    }
}

/// Serves exactly one in-memory layer; everything else is unresolved.
pub(crate) struct MemoryResolver {
    name: String,
    bytes: Vec<u8>,
}

impl Resolver for MemoryResolver {
    fn create_identifier(&self, asset_path: &str, _anchor: Option<&ResolvedPath>) -> String {
        asset_path.to_string()
    }

    fn resolve(&self, asset_path: &str) -> Option<ResolvedPath> {
        (asset_path == self.name).then(|| ResolvedPath::new(&self.name))
    }

    fn resolve_for_new_asset(&self, asset_path: &str) -> Option<ResolvedPath> {
        Some(ResolvedPath::new(asset_path))
    }

    fn open_asset(&self, resolved_path: &ResolvedPath) -> io::Result<Box<dyn Asset>> {
        if resolved_path.to_string_lossy() == self.name {
            Ok(Box::new(io::Cursor::new(self.bytes.clone())))
        } else {
            Err(io::Error::new(
                io::ErrorKind::NotFound,
                resolved_path.to_string(),
            ))
        }
    }

    fn identity(&self) -> String {
        format!("botrail-memory:{}", self.name)
    }
}

/// Serves a fixed set of in-memory layers keyed by normalized relative
/// path, so a multi-file asset — a USD robot and everything it references —
/// composes with no filesystem behind it.
///
/// This is the browser's [`SearchPathResolver`]: wasm cannot open files, and
/// the `Resolver` trait is synchronous so it cannot fetch either. The caller
/// downloads the layer set first (it knows the manifest) and hands the bytes
/// over here.
pub struct BundleResolver {
    layers: HashMap<String, Vec<u8>>,
}

impl BundleResolver {
    /// `layers` maps a layer's path *relative to the bundle root* — the same
    /// spelling used in the references, e.g. `Props/panda_hand.usd` — to its
    /// bytes.
    pub fn new(layers: impl IntoIterator<Item = (String, Vec<u8>)>) -> Self {
        BundleResolver {
            layers: layers
                .into_iter()
                .map(|(name, bytes)| (normalize_bundle_path(&name), bytes))
                .collect(),
        }
    }
}

/// Collapses `./` and `../` segments and drops any scheme + host, so the
/// many spellings of one layer (`./Props/a.usd`, `Props/../Props/a.usd`,
/// `omniverse://host/Props/a.usd`) land on one key.
fn normalize_bundle_path(path: &str) -> String {
    let stripped = SearchPathResolver::strip(path);
    let mut parts: Vec<&str> = Vec::new();
    for segment in stripped.split('/') {
        match segment {
            "" | "." => {}
            ".." => {
                parts.pop();
            }
            other => parts.push(other),
        }
    }
    parts.join("/")
}

impl Resolver for BundleResolver {
    fn create_identifier(&self, asset_path: &str, anchor: Option<&ResolvedPath>) -> String {
        // A reference is written relative to the layer that made it, so an
        // unanchored path is already bundle-relative.
        let anchor_dir = anchor
            .map(|a| a.to_string_lossy().to_string())
            .and_then(|a| a.rsplit_once('/').map(|(dir, _)| dir.to_string()))
            .unwrap_or_default();
        if anchor_dir.is_empty() || asset_path.starts_with('/') {
            return normalize_bundle_path(asset_path);
        }
        normalize_bundle_path(&format!("{anchor_dir}/{asset_path}"))
    }

    fn resolve(&self, asset_path: &str) -> Option<ResolvedPath> {
        let key = normalize_bundle_path(asset_path);
        self.layers
            .contains_key(&key)
            .then(|| ResolvedPath::new(&key))
    }

    fn resolve_for_new_asset(&self, asset_path: &str) -> Option<ResolvedPath> {
        Some(ResolvedPath::new(normalize_bundle_path(asset_path)))
    }

    fn open_asset(&self, resolved_path: &ResolvedPath) -> io::Result<Box<dyn Asset>> {
        let key = normalize_bundle_path(&resolved_path.to_string_lossy());
        match self.layers.get(&key) {
            Some(bytes) => Ok(Box::new(io::Cursor::new(bytes.clone()))),
            None => Err(io::Error::new(io::ErrorKind::NotFound, key)),
        }
    }

    fn identity(&self) -> String {
        format!("botrail-bundle:{}", self.layers.len())
    }
}

/// Rotation mapping Y-up worlds onto botrail's Z-up (+90 deg about X).
pub(crate) fn y_up_to_z_up() -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2)
}

/// The prim's own `primvars:displayColor`, if it is a single constant value.
///
/// botrail shades a whole obstacle uniformly, so only the constant case
/// carries meaning — and only the constant case is the one USD inherits down
/// the namespace. A longer array is per-vertex/per-face data belonging to
/// that one gprim; taking its first element would invent a flat colour the
/// author never asked for and then leak it onto the children, so it is left
/// to the renderer instead.
fn display_color(view: &AnyPrim) -> Option<[f32; 3]> {
    let values: Vec<gf::Vec3f> = view.display_color_attr().get().ok().flatten()?;
    match values.as_slice() {
        [c] => Some([c.x, c.y, c.z]),
        _ => None,
    }
}

struct Importer<'a> {
    stage: &'a Stage,
    meters_per_unit: f64,
    up_axis_fix: UnitQuaternion<f64>,
    /// Resolved lazily in `write_mesh`: the default computation touches
    /// `std::env::temp_dir()`, which panics on wasm (where
    /// `meshes_in_memory` avoids the filesystem entirely).
    mesh_cache_dir: Option<PathBuf>,
    meshes_in_memory: bool,
    out: ImportedScene,
}

impl Importer<'_> {
    fn read_stage_metadata(&mut self) {
        let layer = self.stage.root_layer();
        let Some(root) = layer.pseudo_root() else {
            return;
        };
        if let Ok(Some(v)) = root.field("metersPerUnit") {
            match v {
                sdf::Value::Double(d) => self.meters_per_unit = d,
                sdf::Value::Float(f) => self.meters_per_unit = f as f64,
                _ => {}
            }
        }
        if let Ok(Some(sdf::Value::Token(t))) = root.field("upAxis") {
            if t.as_str() == "Z" {
                self.up_axis_fix = UnitQuaternion::identity();
            }
        }
    }

    fn walk(
        &mut self,
        prim: Prim,
        parent_world: gf::Matrix4d,
        parent_color: Option<[f32; 3]>,
    ) -> anyhow::Result<()> {
        let view = AnyPrim(prim);
        // USD row-vector convention: localToWorld = local * parentLocalToWorld.
        let world = match view.local_to_parent_transform(TimeCode::EARLIEST) {
            Ok(local) => local * parent_world,
            Err(e) => {
                self.warn(view.prim().path(), format!("xform evaluation failed: {e}"));
                parent_world
            }
        };
        // A constant `displayColor` is inherited down the namespace, so a
        // group Xform can paint its whole subtree and a child can override.
        let authored = display_color(&view);
        let color = authored.or(parent_color);

        let prim = view.prim();
        let path = prim.path().to_string();
        // An articulation subtree is a robot, not scenery: importing its
        // links as static obstacles would make the robot self-collide with
        // its own rest-pose geometry. Skip the subtree wholesale and report
        // the root so callers can import it via `import_robot`.
        if prim
            .has_api_schema("PhysicsArticulationRootAPI")
            .unwrap_or(false)
        {
            self.out.robot_roots.push(path);
            return Ok(());
        }
        let visible =
            view.compute_visibility().unwrap_or(Visibility::Inherited) != Visibility::Invisible;
        let renderable = view.compute_purpose().unwrap_or_default() == Purpose::Default;
        let type_name = prim.type_name()?.map(|t| t.to_string()).unwrap_or_default();
        let children = view.prim().children()?;

        if visible && renderable {
            if let Err(e) = self.import_geometry(view.prim(), &path, &type_name, &world, color) {
                self.warn(view.prim().path(), e.to_string());
            }
        }
        // Leaf grouping prims are mount-point markers.
        if children.is_empty() && matches!(type_name.as_str(), "Xform" | "Scope") {
            self.out.frames.push(ImportedFrame {
                name: path,
                pose: self.frame_pose(&world),
            });
        }

        for child in children {
            self.walk(child, world, color)?;
        }
        Ok(())
    }

    fn import_geometry(
        &mut self,
        prim: &Prim,
        path: &str,
        type_name: &str,
        world: &gf::Matrix4d,
        color: Option<[f32; 3]>,
    ) -> anyhow::Result<()> {
        let (pose, residual) = self.normalized_pose(world);
        let node = |geometry, pose| ImportedNode {
            name: path.to_string(),
            geometry,
            pose,
            color,
            mesh_data: None,
        };
        // Per-local-axis scale for primitive dimensions.
        let scale = [
            residual.column(0).norm(),
            residual.column(1).norm(),
            residual.column(2).norm(),
        ];

        match type_name {
            "Mesh" => {
                let Some(mesh) = geom::Mesh::get(self.stage, prim.path().clone())? else {
                    return Ok(());
                };
                let points: Vec<[f32; 3]> = mesh.points_attr().get()?.unwrap_or_default();
                let counts = int_vec(mesh.face_vertex_counts_attr().get::<sdf::Value>()?);
                let face_indices = int_vec(mesh.face_vertex_indices_attr().get::<sdf::Value>()?);
                let data = triangulate(&points, &counts, &face_indices)
                    .map_err(|e| anyhow::anyhow!("mesh: {e}"))?;
                // Bake residual scale/shear + unit conversion into vertices;
                // the rigid part stays in the obstacle pose.
                let mpu = self.meters_per_unit;
                let baked = MeshData {
                    vertices: data
                        .vertices
                        .iter()
                        .map(|v| {
                            let p = residual * Vector3::new(v[0], v[1], v[2]) * mpu;
                            [p.x, p.y, p.z]
                        })
                        .collect(),
                    indices: data.indices,
                    face_colors: Vec::new(),
                };
                if self.meshes_in_memory {
                    self.out.nodes.push(ImportedNode {
                        name: path.to_string(),
                        geometry: Geometry::Mesh {
                            path: PathBuf::from(format!("usd:/{path}")),
                            scale: Vector3::new(1.0, 1.0, 1.0),
                        },
                        pose,
                        color,
                        mesh_data: Some(baked),
                    });
                } else {
                    let stl_path = self.write_mesh(&baked)?;
                    self.out.nodes.push(node(
                        Geometry::Mesh {
                            path: stl_path,
                            scale: Vector3::new(1.0, 1.0, 1.0),
                        },
                        pose,
                    ));
                }
            }
            "Cube" => {
                let size: f64 = geom::Cube::get(self.stage, prim.path().clone())?
                    .and_then(|c| c.size_attr().get().transpose())
                    .transpose()?
                    .unwrap_or(2.0); // USD schema fallback
                let mpu = self.meters_per_unit;
                self.out.nodes.push(node(
                    Geometry::Box {
                        size: Vector3::new(
                            size * scale[0] * mpu,
                            size * scale[1] * mpu,
                            size * scale[2] * mpu,
                        ),
                    },
                    pose,
                ));
            }
            "Sphere" => {
                let radius: f64 = geom::Sphere::get(self.stage, prim.path().clone())?
                    .and_then(|s| s.radius_attr().get().transpose())
                    .transpose()?
                    .unwrap_or(1.0); // USD schema fallback
                if (scale[0] - scale[1]).abs() > 1e-6 || (scale[1] - scale[2]).abs() > 1e-6 {
                    self.warn(
                        prim.path(),
                        "non-uniform scale on Sphere; using the largest axis".to_string(),
                    );
                }
                let s = scale[0].max(scale[1]).max(scale[2]);
                self.out.nodes.push(node(
                    Geometry::Sphere {
                        radius: radius * s * self.meters_per_unit,
                    },
                    pose,
                ));
            }
            "Cylinder" => {
                let cyl = geom::Cylinder::get(self.stage, prim.path().clone())?;
                let Some(cyl) = cyl else { return Ok(()) };
                let radius: f64 = cyl.radius_attr().get()?.unwrap_or(1.0);
                let height: f64 = cyl.height_attr().get()?.unwrap_or(2.0);
                let axis = cyl
                    .axis_attr()
                    .get::<openusd::tf::Token>()?
                    .map(|t| t.to_string())
                    .unwrap_or_else(|| "Z".to_string());
                // botrail cylinders extend along local +z; re-orient other axes.
                let align = match axis.as_str() {
                    "X" => UnitQuaternion::from_axis_angle(
                        &Vector3::y_axis(),
                        std::f64::consts::FRAC_PI_2,
                    ),
                    "Y" => UnitQuaternion::from_axis_angle(
                        &Vector3::x_axis(),
                        -std::f64::consts::FRAC_PI_2,
                    ),
                    _ => UnitQuaternion::identity(),
                };
                let mpu = self.meters_per_unit;
                let s = scale[0].max(scale[1]).max(scale[2]);
                self.out.nodes.push(node(
                    Geometry::Cylinder {
                        radius: radius * s * mpu,
                        length: height * s * mpu,
                    },
                    pose * Isometry3::from_parts(Translation3::identity(), align),
                ));
            }
            // Grouping and non-geometry types pass through silently.
            "Xform" | "Scope" | "" => {}
            "Capsule" | "Cone" | "Plane" | "Points" | "BasisCurves" => {
                anyhow::bail!("unsupported gprim type `{type_name}`; skipped")
            }
            _ => {}
        }
        Ok(())
    }

    /// Splits a USD world matrix (row-vector convention) into a botrail
    /// world pose (meters, Z-up) plus the residual 3x3 (scale/shear, USD
    /// units) to bake into vertices.
    fn normalized_pose(&self, world: &gf::Matrix4d) -> (Isometry3<f64>, Matrix3<f64>) {
        let (raw, residual) = decompose_matrix(world);
        let pose = Isometry3::from_parts(
            Translation3::from(self.up_axis_fix * (raw.translation.vector * self.meters_per_unit)),
            self.up_axis_fix * raw.rotation,
        );
        (pose, residual)
    }

    /// Frame (mount-point) pose. Unlike geometry poses — where the up-axis
    /// fix must rotate authored vertices into the Z-up world — a frame's
    /// orientation is *relabeled* by conjugation, so "identity in a Y-up
    /// world" stays identity and a Z-up robot placed on it stands upright.
    fn frame_pose(&self, world: &gf::Matrix4d) -> Isometry3<f64> {
        let (pose, _) = self.normalized_pose(world);
        Isometry3::from_parts(pose.translation, pose.rotation * self.up_axis_fix.inverse())
    }

    fn write_mesh(&self, mesh: &MeshData) -> Result<PathBuf, io::Error> {
        let dir = self
            .mesh_cache_dir
            .clone()
            .unwrap_or_else(default_mesh_cache_dir);
        write_stl_cached(&dir, mesh)
    }

    fn warn(&mut self, path: &sdf::Path, message: String) {
        self.out.warnings.push(format!("{path}: {message}"));
    }
}

/// Splits a USD matrix (row-vector convention) into its nearest rigid part
/// (raw stage units) and the residual 3x3 (scale/shear) to bake into
/// vertices or primitive dimensions.
pub(crate) fn decompose_matrix(world: &gf::Matrix4d) -> (Isometry3<f64>, Matrix3<f64>) {
    let m = &world.0;
    // Row-vector basis rows become column-vector basis columns.
    let linear = Matrix3::new(
        m[0], m[4], m[8], //
        m[1], m[5], m[9], //
        m[2], m[6], m[10],
    );
    let translation = Vector3::new(m[12], m[13], m[14]);
    // Bounded iteration count: nalgebra's plain `from_matrix` iterates
    // without limit and can spin forever on ill-conditioned matrices
    // (mirrored or near-singular scales — Franka finger links in the
    // wild). Non-convergence just leaves more in the residual, which gets
    // baked into vertices anyway.
    let rotation = if linear.determinant() > 1e-12 {
        Rotation3::from_matrix_eps(&linear, 1e-9, 100, Rotation3::identity())
    } else {
        Rotation3::identity()
    };
    let residual = rotation.inverse().matrix() * linear;
    let pose = Isometry3::from_parts(
        Translation3::from(translation),
        UnitQuaternion::from_rotation_matrix(&rotation),
    );
    (pose, residual)
}

/// Writes a mesh as a content-hash-named binary STL under `dir` (idempotent).
pub(crate) fn write_stl_cached(dir: &Path, mesh: &MeshData) -> Result<PathBuf, io::Error> {
    std::fs::create_dir_all(dir)?;
    let stl = botrail_mesh::to_stl_binary(mesh);
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    stl.hash(&mut hasher);
    let path = dir.join(format!("{:x}-{:016x}.stl", stl.len(), hasher.finish()));
    if !path.exists() {
        std::fs::write(&path, stl)?;
    }
    Ok(path)
}

pub(crate) fn int_vec(value: Option<sdf::Value>) -> Vec<i32> {
    match value {
        Some(sdf::Value::IntVec(v)) => v,
        _ => Vec::new(),
    }
}

/// Fan-triangulates USD face data into an indexed triangle mesh.
pub(crate) fn triangulate(
    points: &[[f32; 3]],
    counts: &[i32],
    face_indices: &[i32],
) -> Result<MeshData, String> {
    if points.is_empty() || counts.is_empty() {
        return Err("no points or faces".to_string());
    }
    let vertices: Vec<[f64; 3]> = points
        .iter()
        .map(|p| [p[0] as f64, p[1] as f64, p[2] as f64])
        .collect();
    let mut indices = Vec::new();
    let mut at = 0usize;
    for &count in counts {
        let count = count as usize;
        let face = face_indices
            .get(at..at + count)
            .ok_or("faceVertexIndices shorter than faceVertexCounts")?;
        for k in 1..count.saturating_sub(1) {
            let tri = [face[0], face[k], face[k + 1]];
            if tri.iter().any(|&i| i < 0 || i as usize >= vertices.len()) {
                return Err(format!("face index out of range: {tri:?}"));
            }
            indices.push([tri[0] as u32, tri[1] as u32, tri[2] as u32]);
        }
        at += count;
    }
    if indices.is_empty() {
        return Err("no triangles".to_string());
    }
    Ok(MeshData::new(vertices, indices))
}

pub(crate) fn default_mesh_cache_dir() -> PathBuf {
    let base = std::env::var_os("BOTRAIL_CACHE_DIR")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".cache").join("botrail"))
        })
        .unwrap_or_else(|| std::env::temp_dir().join("botrail-cache"));
    base.join("usd-meshes")
}

// -------------------------------------------------------------- resolver

/// Filesystem resolver that additionally strips `omniverse://host/` and
/// `http(s)://host/` schemes so Isaac-style references resolve against the
/// local search paths (a downloaded asset pack).
pub(crate) struct SearchPathResolver {
    inner: DefaultResolver,
}

impl SearchPathResolver {
    pub(crate) fn new(search_paths: Vec<PathBuf>) -> Self {
        SearchPathResolver {
            inner: DefaultResolver::with_search_paths(search_paths),
        }
    }

    pub(crate) fn strip(asset_path: &str) -> &str {
        for scheme in ["omniverse://", "http://", "https://"] {
            if let Some(rest) = asset_path.strip_prefix(scheme) {
                // Drop the host segment; keep the path relative.
                return rest.split_once('/').map(|(_, p)| p).unwrap_or("");
            }
        }
        asset_path
    }
}

impl Resolver for SearchPathResolver {
    fn create_identifier(&self, asset_path: &str, anchor: Option<&ResolvedPath>) -> String {
        let stripped = Self::strip(asset_path);
        // A remote reference is anchored to nothing local; identify it by
        // its stripped path so equal references dedupe.
        if stripped != asset_path {
            return stripped.to_string();
        }
        let identifier = self.inner.create_identifier(asset_path, anchor);
        // The inner identifier canonicalizes absolute paths, following
        // symlinks — see `preserve_link_name`. Rebuild the
        // pre-canonicalization path (anchoring relative references the way
        // the inner resolver does) so the link's own name survives into the
        // identifier; `resolve` applies the same preservation again.
        let path = Path::new(asset_path);
        let anchored = if path.is_absolute() {
            path.to_path_buf()
        } else if let Some(anchor) = anchor {
            match AsRef::<Path>::as_ref(anchor).parent() {
                Some(dir) if !dir.as_os_str().is_empty() => dir.join(path),
                _ => return identifier,
            }
        } else {
            return identifier;
        };
        preserve_link_name(&anchored.to_string_lossy(), ResolvedPath::new(&identifier))
            .display()
            .to_string()
    }

    fn resolve(&self, asset_path: &str) -> Option<ResolvedPath> {
        let stripped = Self::strip(asset_path);
        Some(preserve_link_name(stripped, self.inner.resolve(stripped)?))
    }

    fn resolve_for_new_asset(&self, asset_path: &str) -> Option<ResolvedPath> {
        self.inner.resolve_for_new_asset(Self::strip(asset_path))
    }

    fn open_asset(&self, resolved_path: &ResolvedPath) -> io::Result<Box<dyn Asset>> {
        self.inner.open_asset(resolved_path)
    }

    fn identity(&self) -> String {
        format!("botrail-search:{}", self.inner.identity())
    }
}

/// The default resolver canonicalizes, which follows a symlink all the way
/// to its target — and a target named differently from the link loses the
/// link's extension, taking the file-format dispatch with it. The
/// huggingface_hub cache is the motivating case: `usd/model.usdc` is a
/// symlink onto an extensionless content-addressed blob. Keep the link's
/// own file name, anchored to its canonicalized directory.
fn preserve_link_name(asset_path: &str, resolved: ResolvedPath) -> ResolvedPath {
    let asset = Path::new(asset_path);
    let Some(name) = asset.file_name() else {
        return resolved;
    };
    if resolved.file_name() == Some(name) {
        return resolved;
    }
    let relinked = asset
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .and_then(|parent| parent.canonicalize().ok())
        .map(|dir| dir.join(name));
    match relinked {
        Some(path) if path.exists() => ResolvedPath::new(path),
        _ => resolved,
    }
}

impl ImportedScene {
    pub fn frame(&self, name: &str) -> Option<&ImportedFrame> {
        self.frames.iter().find(|f| f.name == name)
    }
}

/// Filesystem layers a stage transitively loads (root, sublayers,
/// reference/payload targets) — the set to bundle when archiving a
/// USD-sourced robot. Opens the stage and traverses it fully so on-demand
/// arcs are forced to load.
pub fn stage_dependencies(
    path: &Path,
    search_paths: &[PathBuf],
) -> Result<Vec<PathBuf>, UsdImportError> {
    let mut search_paths = search_paths.to_vec();
    if let Some(dir) = path.parent() {
        search_paths.push(dir.to_path_buf());
    }
    let stage = Stage::builder()
        .resolver(SearchPathResolver::new(search_paths))
        .open(&path.display().to_string())
        .map_err(|e| UsdImportError::Open {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;

    fn force_load(prim: Prim) -> anyhow::Result<()> {
        for child in prim.children()? {
            force_load(child)?;
        }
        Ok(())
    }
    force_load(stage.prim(sdf::Path::abs_root()))
        .map_err(|e| UsdImportError::Traverse(e.to_string()))?;

    Ok(stage
        .layer_identifiers()
        .into_iter()
        .map(PathBuf::from)
        .filter(|p| p.is_file())
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A Y-up, centimeter-unit cell: a referenced table (mesh + cube),
    /// instanced bins, a variant fixture, a guide-purpose marker, and a
    /// mount frame. Exercises composition, filtering, and normalization.
    const CELL: &str = r#"#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "World"
{
    def Xform "Table" (prepend references = @./table.usda@</Table>)
    {
        double3 xformOp:translate = (100, 0, 50)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def "Bin_A" (instanceable = true, prepend references = @./bin.usda@</Bin>)
    {
        double3 xformOp:translate = (30, 80, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def Sphere "Ball"
    {
        double radius = 5
        double3 xformOp:translate = (0, 20, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def Sphere "FallbackBall"
    {
    }

    def Xform "Fixture" (
        prepend variantSets = "size"
        variants = { string size = "large" }
    )
    {
        variantSet "size" = {
            "small" { def Cube "Body" { double size = 10 } }
            "large" { def Cube "Body" { double size = 25 } }
        }
    }

    def Scope "Guides"
    {
        def Sphere "Marker"
        {
            uniform token purpose = "guide"
            double radius = 99
        }
    }

    def Xform "MountFrame"
    {
        double3 xformOp:translate = (10, 75, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"#;

    const TABLE: &str = r#"#usda 1.0
(
    defaultPrim = "Table"
)

def Xform "Table"
{
    def Mesh "Top"
    {
        point3f[] points = [(-60, 70, -40), (60, 70, -40), (60, 70, 40), (-60, 70, 40), (-60, 75, -40), (60, 75, -40), (60, 75, 40), (-60, 75, 40)]
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 1, 5, 6, 2, 2, 6, 7, 3, 3, 7, 4, 0]
    }

    def Cube "Leg"
    {
        double size = 10
        double3 xformOp:translate = (0, 35, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"#;

    const BIN: &str = r#"#usda 1.0
(
    defaultPrim = "Bin"
)

def Xform "Bin"
{
    def Cube "Shell"
    {
        double size = 20
    }
}
"#;

    fn write_cell(dir: &Path) -> PathBuf {
        std::fs::create_dir_all(dir).unwrap();
        std::fs::write(dir.join("table.usda"), TABLE).unwrap();
        std::fs::write(dir.join("bin.usda"), BIN).unwrap();
        let cell = dir.join("cell.usda");
        std::fs::write(&cell, CELL).unwrap();
        cell
    }

    fn import_cell() -> (ImportedScene, PathBuf) {
        let dir = std::env::temp_dir().join(format!("botrail-usd-test-{}", std::process::id()));
        let cell = write_cell(&dir);
        let options = ImportOptions {
            mesh_cache_dir: Some(dir.join("meshes")),
            ..Default::default()
        };
        (import_usd(&cell, &options).unwrap(), dir)
    }

    /// A Z-up cell containing scenery AND an articulated robot: the robot
    /// subtree must be skipped (reported via `robot_roots`), the crate kept.
    const CELL_WITH_ROBOT: &str = r#"#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def Cube "Crate"
    {
        double size = 0.2
    }

    def Xform "Robot" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
    {
        def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
        {
            def Cube "geom"
            {
                double size = 0.1
            }
        }

        def Xform "Mount"
        {
        }
    }

    def Xform "PickFrame"
    {
    }
}
"#;

    #[test]
    fn articulation_subtrees_are_skipped_and_reported() {
        let dir =
            std::env::temp_dir().join(format!("botrail-usd-robotskip-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("cell.usda");
        std::fs::write(&path, CELL_WITH_ROBOT).unwrap();
        let scene = import_usd(&path, &ImportOptions::default()).unwrap();
        let _ = std::fs::remove_dir_all(&dir);

        // The robot's link geometry did NOT become an obstacle, and nothing
        // under the robot became a frame — but the rest of the cell did.
        assert_eq!(scene.robot_roots, vec!["/World/Robot"]);
        assert!(!scene
            .nodes
            .iter()
            .any(|n| n.name.starts_with("/World/Robot")));
        assert!(scene
            .frames
            .iter()
            .all(|f| !f.name.starts_with("/World/Robot")));
        assert_eq!(scene.nodes.len(), 1);
        assert_eq!(scene.nodes[0].name, "/World/Crate");
        assert!(scene.frames.iter().any(|f| f.name == "/World/PickFrame"));
    }

    fn node<'a>(scene: &'a ImportedScene, name: &str) -> &'a ImportedNode {
        scene
            .nodes
            .iter()
            .find(|n| n.name == name)
            .unwrap_or_else(|| panic!("missing node {name}: {:?}", scene.nodes))
    }

    #[test]
    fn imports_composed_scene_normalized_to_meters_z_up() {
        let (scene, dir) = import_cell();

        // Referenced cube leg: cm -> m, Y-up -> Z-up ((x, y, z) -> (x, -z, y)).
        // USD world (100, 35, 50)cm -> (1.0, -0.5, 0.35)m.
        let leg = node(&scene, "/World/Table/Leg");
        let t = leg.pose.translation;
        assert!((t.x - 1.0).abs() < 1e-9, "{t:?}");
        assert!((t.y + 0.5).abs() < 1e-9, "{t:?}");
        assert!((t.z - 0.35).abs() < 1e-9, "{t:?}");
        assert!(matches!(leg.geometry, Geometry::Box { size } if (size.x - 0.1).abs() < 1e-9));

        // Referenced mesh: extracted to a cached STL with baked cm -> m.
        let top = node(&scene, "/World/Table/Top");
        let Geometry::Mesh { path, .. } = &top.geometry else {
            panic!("table top should be a mesh: {:?}", top.geometry)
        };
        let mesh = botrail_mesh::load_path(path).unwrap();
        assert_eq!(mesh.indices.len(), 12); // 6 quads fan into 12 tris
        let max = mesh.vertices.iter().map(|v| v[0]).fold(f64::MIN, f64::max);
        assert!((max - 0.6).abs() < 1e-6, "x max {max} (60cm -> 0.6m)");

        // Instanced prim's child resolves with the instance transform.
        let shell = node(&scene, "/World/Bin_A/Shell");
        assert!((shell.pose.translation.z - 0.8).abs() < 1e-9);

        // Variant selection: size=large -> 25cm cube.
        let body = node(&scene, "/World/Fixture/Body");
        assert!(matches!(body.geometry, Geometry::Box { size } if (size.x - 0.25).abs() < 1e-9));

        // Schema-fallback sphere radius (1 unit = 1cm).
        let fallback = node(&scene, "/World/FallbackBall");
        assert!(
            matches!(fallback.geometry, Geometry::Sphere { radius } if (radius - 0.01).abs() < 1e-9)
        );

        // Guide-purpose marker is filtered out.
        assert!(scene.frame("/World/Guides/Marker").is_none());
        assert!(!scene.nodes.iter().any(|n| n.name.contains("Marker")));

        // Leaf Xform becomes a frame at the normalized pose; identity
        // orientation in the Y-up world stays identity (conjugated fix), so
        // robots placed on it stand upright.
        let frame = scene.frame("/World/MountFrame").expect("mount frame");
        assert!((frame.pose.translation.x - 0.1).abs() < 1e-9);
        assert!((frame.pose.translation.z - 0.75).abs() < 1e-9);
        assert!(frame.pose.rotation.angle() < 1e-9);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn z_up_meter_stages_pass_through() {
        let dir = std::env::temp_dir().join(format!("botrail-usd-zup-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let usda = r#"#usda 1.0
(
    defaultPrim = "W"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "W" {
    def Cube "C" {
        double size = 0.5
        double3 xformOp:translate = (1, 2, 3)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"#;
        let path = dir.join("zup.usda");
        std::fs::write(&path, usda).unwrap();
        let scene = import_usd(&path, &ImportOptions::default()).unwrap();
        let c = node(&scene, "/W/C");
        assert!((c.pose.translation.vector - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-9);
        assert!(matches!(c.geometry, Geometry::Box { size } if (size.z - 0.5).abs() < 1e-12));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn display_color_is_read_and_inherited() {
        let dir = std::env::temp_dir().join(format!("botrail-usd-color-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let usda = r#"#usda 1.0
(
    defaultPrim = "W"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "W" {
    def Xform "Painted" {
        color3f[] primvars:displayColor = [(0.25, 0.5, 0.75)]

        def Cube "Inherits" { double size = 0.1 }

        def Cube "Overrides" {
            double size = 0.1
            color3f[] primvars:displayColor = [(1, 0, 0)]
        }

        def Cube "PerVertex" {
            double size = 0.1
            color3f[] primvars:displayColor = [(1, 0, 0), (0, 1, 0)]
        }
    }

    def Cube "Bare" { double size = 0.1 }
}
"#;
        let path = dir.join("color.usda");
        std::fs::write(&path, usda).unwrap();
        let scene = import_usd(&path, &ImportOptions::default()).unwrap();

        assert_eq!(
            node(&scene, "/W/Painted/Inherits").color,
            Some([0.25, 0.5, 0.75])
        );
        assert_eq!(
            node(&scene, "/W/Painted/Overrides").color,
            Some([1.0, 0.0, 0.0])
        );
        // A multi-element array is per-vertex data, not a flat colour for the
        // whole prim — it falls back to what the group painted.
        assert_eq!(
            node(&scene, "/W/Painted/PerVertex").color,
            Some([0.25, 0.5, 0.75])
        );
        assert_eq!(node(&scene, "/W/Bare").color, None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn scheme_stripping_resolves_against_search_paths() {
        assert_eq!(
            SearchPathResolver::strip("omniverse://localhost/Isaac/Props/box.usd"),
            "Isaac/Props/box.usd"
        );
        assert_eq!(
            SearchPathResolver::strip("https://assets.example/pack/a.usd"),
            "pack/a.usd"
        );
        assert_eq!(SearchPathResolver::strip("./local.usda"), "./local.usda");
    }
}

#[cfg(test)]
mod bytes_tests {
    use super::*;

    #[test]
    fn in_memory_import_keeps_meshes_off_disk() {
        let usda = r#"#usda 1.0
(
    defaultPrim = "W"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "W" {
    def Mesh "Tri" {
        point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        int[] faceVertexCounts = [3]
        int[] faceVertexIndices = [0, 1, 2]
    }
    def Cube "C" { double size = 0.5 }
}
"#;
        let dir = std::env::temp_dir().join(format!("botrail-usd-mem-{}", std::process::id()));
        let options = ImportOptions {
            meshes_in_memory: true,
            mesh_cache_dir: Some(dir.clone()),
            ..Default::default()
        };
        let scene = import_usd_bytes(usda.as_bytes().to_vec(), "drop.usda", &options).unwrap();

        let mesh = scene.nodes.iter().find(|n| n.name == "/W/Tri").unwrap();
        let data = mesh.mesh_data.as_ref().expect("mesh kept in memory");
        assert_eq!(data.indices.len(), 1);
        assert!(matches!(&mesh.geometry, Geometry::Mesh { path, .. }
            if path.to_string_lossy().starts_with("usd:/")));
        // Nothing was written to the cache dir.
        assert!(!dir.exists());

        // Primitives import as usual.
        let cube = scene.nodes.iter().find(|n| n.name == "/W/C").unwrap();
        assert!(matches!(cube.geometry, Geometry::Box { size } if (size.x - 0.5).abs() < 1e-9));
        assert!(cube.mesh_data.is_none());
    }
}
