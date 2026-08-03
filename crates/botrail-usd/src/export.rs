//! USD animation export: bake robot link + object motion into a USD layer
//! as `xformOp` timeSamples.
//!
//! Each robot lands under `/World/<sanitized instance name>` (single-robot
//! exporters pass `"Robot"`, keeping the historical layer shape). Two robot
//! paths:
//! - **USD-sourced robots** reference the original stage and override the
//!   body prims with per-frame local transforms, keeping full visual
//!   fidelity (materials, meshes) without re-authoring anything. Robots
//!   sharing a source stage share one copied asset directory.
//! - **URDF robots** are authored from scratch: flat link Xforms under the
//!   robot prim with the model's visual shapes as child gprims.
//!
//! Obstacles land under `/World/Env` (their `/`-segmented names become
//! nested prims); grasped objects get sampled tracks, the rest are static.
//!
//! # Frame conversion (the importer's inverse)
//!
//! The importer maps raw stage coords to botrail (Z-up, meters) with
//! `t' = F·(t·mpu)`, `R' = F·R·F⁻¹` (`F` = +90° about X on Y-up stages) and
//! re-frames every body by its inbound joint's `localPose1` (`K`): botrail
//! link frames are URDF-style *joint* frames, `X_stage←body = X_stage←link ∘
//! K⁻¹`. Export inverts both: FK world poses go through `t = F⁻¹·t'/mpu`,
//! `R = F⁻¹·R'·F`, then `∘ K⁻¹` onto the body prim — authored *relative to
//! the prim's parent* (nearest animated body ancestor, or the reference
//! root, with any static intermediate transforms folded in).
//!
//! The animation layer is Z-up/meters; every robot prim carries a
//! corrective `orient = F⁻¹→export, scale = mpu` pair so non-Z-up or
//! non-meter robot stages compose correctly (identity for Isaac-style
//! Z-up/meter assets).

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use botrail_model::{Geometry, RobotModel, RobotSource, Shape};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use openusd::schemas::geom::Xformable;
use openusd::schemas::physics::JointBase;
use openusd::sdf::{
    self, ChildrenKey, FieldKey, ListOp, Reference, SpecType, Specifier, Value, Variability,
};
use openusd::usd::{SchemaBase, Stage, TimeCode};
use openusd::usda::TextWriter;
use openusd::{gf, tf};
use thiserror::Error;

use crate::articulation::AnyJoint;
use crate::{decompose_matrix, y_up_to_z_up, AnyPrim, SearchPathResolver};

#[derive(Debug, Error)]
pub enum UsdExportError {
    #[error("failed to open robot stage `{path}`: {message}")]
    Open { path: String, message: String },
    #[error("robot stage export: {0}")]
    RobotStage(String),
    #[error("invalid export input: {0}")]
    Input(String),
    #[error("usd authoring failed: {0}")]
    Author(String),
    #[error("mesh `{path}`: {message}")]
    Mesh { path: String, message: String },
    #[error("{0}")]
    Io(String),
}

#[derive(Debug, Clone)]
pub struct ExportOptions {
    /// Time codes per second of the layer; samples land on `t * fps`.
    pub fps: f64,
}

impl Default for ExportOptions {
    fn default() -> Self {
        ExportOptions { fps: 60.0 }
    }
}

/// World-pose motion of one exported object (an obstacle).
pub enum PoseTrack {
    Static(Isometry3<f64>),
    /// One world pose per frame, aligned with `AnimationInput::times`.
    Sampled(Vec<Isometry3<f64>>),
}

pub struct ObjectSpec {
    /// Obstacle name; `/`-segments become nested prims under `/World/Env`.
    pub name: String,
    pub geometry: Geometry,
    pub track: PoseTrack,
    /// `primvars:displayColor` to author, linear RGB. `None` falls back to
    /// the neutral environment grey.
    pub color: Option<[f32; 3]>,
    /// One flag per animation frame: false hides the prim that frame.
    /// Empty means always visible. USD carries this natively as animated
    /// `visibility`, so a magazine full of stock stays out of the picture
    /// in usdview exactly as it does in the studio.
    pub visible: Vec<bool>,
}

/// One robot's animation bundle: the instance names the prim under
/// `/World` (sanitized) and the asset directory.
pub struct RobotAnimation<'a> {
    /// Scene instance name. Single-robot exporters pass `"Robot"` so the
    /// layer keeps its pre-multi-robot shape (`/World/Robot`,
    /// `<stem>_assets/robot/`) byte for byte.
    pub name: &'a str,
    pub model: &'a RobotModel,
    /// World pose of every link per frame (botrail Z-up meters), aligned
    /// with `AnimationInput::times`; inner vectors align with `model.links`.
    pub link_poses: &'a [Vec<Isometry3<f64>>],
    /// Joint positions per frame (rad / m, DOF order). When present and the
    /// robot is USD-sourced, `JointStateAPI` timeSamples are authored on the
    /// joint prims alongside the link transforms, so readers (botrail
    /// included) can recover q(t) without projecting link poses.
    pub joint_samples: Option<&'a [Vec<f64>]>,
}

pub struct AnimationInput<'a> {
    /// One bundle per robot, in scene order.
    pub robots: &'a [RobotAnimation<'a>],
    /// Frame times in seconds, strictly increasing, starting at 0 — shared
    /// by every robot and sampled object track.
    pub times: &'a [f64],
    pub objects: &'a [ObjectSpec],
}

pub struct ExportedAnimation {
    /// The animation layer, serialized as usda text.
    pub usda: String,
    /// Files to place next to the written layer (absolute source, relative
    /// destination) so the robot reference resolves — empty for URDF robots.
    pub assets: Vec<(PathBuf, PathBuf)>,
    pub warnings: Vec<String>,
}

/// Exports and writes the layer to `path` (plus referenced robot assets in
/// a sibling `<stem>_assets/` directory). Returns accumulated warnings.
pub fn write_animation(
    path: &Path,
    input: &AnimationInput,
    options: &ExportOptions,
) -> Result<Vec<String>, UsdExportError> {
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "animation".to_string());
    let exported = export_animation(input, options, &stem)?;
    let io = |e: std::io::Error| UsdExportError::Io(e.to_string());
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(io)?;
    }
    std::fs::write(path, &exported.usda).map_err(io)?;
    let base = path.parent().unwrap_or(Path::new(""));
    for (src, rel) in &exported.assets {
        let dest = base.join(rel);
        if let Some(dir) = dest.parent() {
            std::fs::create_dir_all(dir).map_err(io)?;
        }
        std::fs::copy(src, &dest).map_err(io)?;
    }
    Ok(exported.warnings)
}

/// Builds the animation layer in memory. `asset_stem` names the sibling
/// asset directory referenced by USD-sourced robots (`<stem>_assets/`).
pub fn export_animation(
    input: &AnimationInput,
    options: &ExportOptions,
    asset_stem: &str,
) -> Result<ExportedAnimation, UsdExportError> {
    let n = input.times.len();
    if n == 0 {
        return Err(UsdExportError::Input("no frames".into()));
    }
    if input.robots.is_empty() {
        return Err(UsdExportError::Input("no robots".into()));
    }
    for robot in input.robots {
        if robot.link_poses.len() != n {
            return Err(UsdExportError::Input(format!(
                "robot `{}`: {} frames but {} link pose sets",
                robot.name,
                n,
                robot.link_poses.len()
            )));
        }
        for (f, poses) in robot.link_poses.iter().enumerate() {
            if poses.len() != robot.model.links.len() {
                return Err(UsdExportError::Input(format!(
                    "robot `{}` frame {f}: {} link poses for {} links",
                    robot.name,
                    poses.len(),
                    robot.model.links.len()
                )));
            }
        }
        if let Some(joint_samples) = robot.joint_samples {
            if joint_samples.len() != n {
                return Err(UsdExportError::Input(format!(
                    "robot `{}`: {} frames but {} joint sample sets",
                    robot.name,
                    n,
                    joint_samples.len()
                )));
            }
            if joint_samples.iter().any(|q| q.len() != robot.model.dof()) {
                return Err(UsdExportError::Input(format!(
                    "robot `{}`: joint samples must have {} values (model DOF)",
                    robot.name,
                    robot.model.dof()
                )));
            }
        }
    }
    for obj in input.objects {
        if let PoseTrack::Sampled(samples) = &obj.track {
            if samples.len() != n {
                return Err(UsdExportError::Input(format!(
                    "object `{}`: {} samples for {} frames",
                    obj.name,
                    samples.len(),
                    n
                )));
            }
        }
    }

    let fps = options.fps;
    let codes: Vec<f64> = input.times.iter().map(|t| t * fps).collect();
    let mut warnings = Vec::new();
    let mut layer = LayerBuilder::new();

    let root_fields = [
        (FieldKey::DefaultPrim.as_ref(), Value::Token("World".into())),
        (
            FieldKey::Documentation.as_ref(),
            Value::String("Baked robot animation generated by botrail".into()),
        ),
        (FieldKey::StartTimeCode.as_ref(), Value::Double(codes[0])),
        (
            FieldKey::EndTimeCode.as_ref(),
            Value::Double(*codes.last().expect("n > 0")),
        ),
        (FieldKey::TimeCodesPerSecond.as_ref(), Value::Double(fps)),
        (FieldKey::FramesPerSecond.as_ref(), Value::Double(fps)),
        ("metersPerUnit", Value::Double(1.0)),
        ("upAxis", Value::Token("Z".into())),
    ];
    for (key, value) in root_fields {
        layer.root_field(key, value);
    }
    layer.ensure_prim("/World", Specifier::Def, Some("Xform"));

    let mut assets = Vec::new();
    // Robot prims under /World, uniquified from sanitized instance names;
    // asset directories dedup by source stage (two instances of the same
    // asset share one copy, both references point at it).
    let mut used_prims: HashMap<String, usize> = HashMap::new();
    let mut used_dirs: HashMap<String, usize> = HashMap::new();
    let mut source_dirs: HashMap<PathBuf, String> = HashMap::new();
    for robot in input.robots {
        let prim_name = unique_child(&mut used_prims, &sanitize_name(robot.name));
        let robot_prim = format!("/World/{prim_name}");
        match &robot.model.source {
            RobotSource::Usd {
                path,
                articulation_root,
            } => {
                let (info, _stage) =
                    robot_stage_info(path, &[], articulation_root, robot.model, &mut warnings)?;
                let dir = match source_dirs.get(path) {
                    Some(dir) => dir.clone(),
                    None => {
                        let dir =
                            unique_child(&mut used_dirs, &sanitize_name(robot.name).to_lowercase());
                        source_dirs.insert(path.clone(), dir.clone());
                        assets.extend(robot_asset_copies(path, asset_stem, &dir, &mut warnings)?);
                        dir
                    }
                };
                author_referenced_robot(
                    &mut layer,
                    robot,
                    &codes,
                    &info,
                    &robot_prim,
                    &format!("./{asset_stem}_assets/{dir}"),
                    &mut warnings,
                )?;
            }
            RobotSource::UrdfXml(_) => {
                author_urdf_robot(&mut layer, robot, &codes, &robot_prim, &mut warnings)?;
            }
        }
    }

    author_objects(&mut layer, input.objects, &codes, &mut warnings)?;

    Ok(ExportedAnimation {
        usda: layer.finish()?,
        assets,
        warnings,
    })
}

// ------------------------------------------------------------ conversions

/// Source-stage normalization: botrail world = `S(stage)` with
/// `t' = F·(t·mpu)`, `R' = F·R·F⁻¹`.
#[derive(Clone, Copy)]
pub(crate) struct StageFrame {
    pub(crate) mpu: f64,
    pub(crate) up_fix: UnitQuaternion<f64>,
}

impl StageFrame {
    /// Maps a botrail-world pose into raw source-stage coordinates
    /// (the importer conjugation's inverse).
    fn to_stage(self, x: &Isometry3<f64>) -> Isometry3<f64> {
        let fi = self.up_fix.inverse();
        Isometry3::from_parts(
            Translation3::from((fi * x.translation.vector) / self.mpu),
            fi * x.rotation * self.up_fix,
        )
    }
}

fn translate_value(x: &Isometry3<f64>) -> Value {
    let t = x.translation;
    Value::Vec3d(gf::vec3d(t.x, t.y, t.z))
}

fn orient_value(x: &Isometry3<f64>) -> Value {
    let q = x.rotation;
    Value::Quatd(gf::quatd(q.w, q.i, q.j, q.k))
}

/// openusd's text writer prints single-item list-ops without brackets.
/// That shorthand is fine for `references`, but pxr's parser insists on a
/// bracketed list for `apiSchemas` — wrap those lines so exported layers
/// open in stock USD tooling.
fn bracket_singleton_api_schemas(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for line in text.split_inclusive('\n') {
        let body = line.trim_end_matches(['\n', '\r']);
        let trimmed = body.trim_start();
        let is_scalar = ["prepend ", "append ", "add ", "delete ", ""]
            .iter()
            .any(|kw| {
                trimmed
                    .strip_prefix(kw)
                    .and_then(|r| r.strip_prefix("apiSchemas = \""))
                    .is_some_and(|r| r.ends_with('"'))
            });
        if is_scalar {
            let eq = body.find("= ").expect("matched above");
            out.push_str(&body[..eq + 2]);
            out.push('[');
            out.push_str(&body[eq + 2..]);
            out.push(']');
            out.push_str(&line[body.len()..]);
        } else {
            out.push_str(line);
        }
    }
    out
}

/// USD prim names allow `[A-Za-z_][A-Za-z0-9_]*`.
pub(crate) fn sanitize_name(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    for (i, c) in name.chars().enumerate() {
        let valid = c.is_ascii_alphanumeric() || c == '_';
        let leading_digit = i == 0 && c.is_ascii_digit();
        if !valid || leading_digit {
            if i == 0 {
                out.push('_');
            }
            if valid {
                out.push(c);
            } else {
                out.push('_');
            }
        } else {
            out.push(c);
        }
    }
    if out.is_empty() {
        out.push('_');
    }
    out
}

// -------------------------------------------------------- layer authoring

/// Thin authoring wrapper over raw [`sdf::Data`]: keeps the child/property
/// name bookkeeping the text writer needs.
struct LayerBuilder {
    data: sdf::Data,
    prim_children: HashMap<String, Vec<String>>,
    prop_children: HashMap<String, Vec<String>>,
    prims: HashMap<String, Specifier>,
}

impl LayerBuilder {
    fn new() -> Self {
        let mut data = sdf::Data::new();
        data.create_spec(sdf::Path::abs_root(), SpecType::PseudoRoot);
        LayerBuilder {
            data,
            prim_children: HashMap::new(),
            prop_children: HashMap::new(),
            prims: HashMap::new(),
        }
    }

    fn root_field(&mut self, key: &str, value: Value) {
        let spec = self
            .data
            .spec_mut(&sdf::Path::abs_root())
            .expect("pseudo-root created in new()");
        spec.add(key, value);
    }

    /// Creates the prim spec (and missing ancestors as `over`) if absent.
    /// A later `Def` request upgrades an existing `over`.
    fn ensure_prim(&mut self, path: &str, specifier: Specifier, type_name: Option<&str>) {
        debug_assert!(path.starts_with('/') && path.len() > 1);
        let (parent, name) = match path.rfind('/') {
            Some(0) => ("".to_string(), &path[1..]),
            Some(i) => (path[..i].to_string(), &path[i + 1..]),
            None => unreachable!("absolute prim path"),
        };
        if !parent.is_empty() && !self.prims.contains_key(&parent) {
            self.ensure_prim(&parent, Specifier::Over, None);
        }
        let parent_key = if parent.is_empty() {
            "/".to_string()
        } else {
            parent.clone()
        };
        let children = self.prim_children.entry(parent_key).or_default();
        if !children.iter().any(|c| c == name) {
            children.push(name.to_string());
        }

        match self.prims.get(path) {
            None => {
                let spec = self.data.create_spec(
                    sdf::path(path).expect("sanitized prim path"),
                    SpecType::Prim,
                );
                spec.add(FieldKey::Specifier, Value::Specifier(specifier));
                if let Some(t) = type_name {
                    spec.add(FieldKey::TypeName, Value::Token(t.into()));
                }
                self.prims.insert(path.to_string(), specifier);
            }
            Some(Specifier::Over) if specifier == Specifier::Def => {
                let spec = self
                    .data
                    .spec_mut(&sdf::path(path).expect("sanitized prim path"))
                    .expect("spec exists");
                spec.add(FieldKey::Specifier, Value::Specifier(Specifier::Def));
                if let Some(t) = type_name {
                    spec.add(FieldKey::TypeName, Value::Token(t.into()));
                }
                self.prims.insert(path.to_string(), Specifier::Def);
            }
            Some(_) => {}
        }
    }

    fn has_prim(&self, path: &str) -> bool {
        self.prims.contains_key(path)
    }

    fn prim_field(&mut self, path: &str, key: FieldKey, value: Value) {
        let spec = self
            .data
            .spec_mut(&sdf::path(path).expect("sanitized prim path"))
            .expect("prim created via ensure_prim");
        spec.add(key, value);
    }

    fn attr(&mut self, prim: &str, name: &str, type_name: &str, value: AttrValue) {
        let props = self.prop_children.entry(prim.to_string()).or_default();
        if !props.iter().any(|p| p == name) {
            props.push(name.to_string());
        }
        let path = sdf::path(prim)
            .expect("sanitized prim path")
            .append_property(name)
            .expect("valid property name");
        let spec = self.data.create_spec(path, SpecType::Attribute);
        spec.add(FieldKey::TypeName, Value::Token(type_name.into()));
        match value {
            AttrValue::Default(v) => spec.add(FieldKey::Default, v),
            AttrValue::Uniform(v) => {
                spec.add(
                    FieldKey::Variability,
                    Value::Variability(Variability::Uniform),
                );
                spec.add(FieldKey::Default, v);
            }
            AttrValue::Samples(map) => spec.add(FieldKey::TimeSamples, Value::TimeSamples(map)),
        }
    }

    /// Authors translate/orient ops (samples or static) on a prim,
    /// replacing its op order. `extra_scale` appends a static scale op.
    fn xform(&mut self, prim: &str, pose: &XformValue, extra_scale: Option<[f64; 3]>) {
        let mut order = vec![
            "xformOp:translate".to_string(),
            "xformOp:orient".to_string(),
        ];
        match pose {
            XformValue::Static(x) => {
                self.attr(
                    prim,
                    "xformOp:translate",
                    "double3",
                    AttrValue::Default(translate_value(x)),
                );
                self.attr(
                    prim,
                    "xformOp:orient",
                    "quatd",
                    AttrValue::Default(orient_value(x)),
                );
            }
            XformValue::Sampled(codes, poses) => {
                let translate: sdf::TimeSampleMap = codes
                    .iter()
                    .zip(poses.iter())
                    .map(|(c, x)| (*c, translate_value(x)))
                    .collect();
                let orient: sdf::TimeSampleMap = codes
                    .iter()
                    .zip(poses.iter())
                    .map(|(c, x)| (*c, orient_value(x)))
                    .collect();
                self.attr(
                    prim,
                    "xformOp:translate",
                    "double3",
                    AttrValue::Samples(translate),
                );
                self.attr(prim, "xformOp:orient", "quatd", AttrValue::Samples(orient));
            }
        }
        if let Some(s) = extra_scale {
            // double3, not the customary float3: the unit-correction scale
            // (metersPerUnit) must survive exactly for the FK round-trip.
            self.attr(
                prim,
                "xformOp:scale",
                "double3",
                AttrValue::Default(Value::Vec3d(gf::vec3d(s[0], s[1], s[2]))),
            );
            order.push("xformOp:scale".to_string());
        }
        self.attr(
            prim,
            "xformOpOrder",
            "token[]",
            AttrValue::Uniform(Value::token_vec(order)),
        );
    }

    fn finish(mut self) -> Result<String, UsdExportError> {
        let mut children: Vec<(String, Vec<String>)> = self.prim_children.drain().collect();
        for (parent, names) in children.drain(..) {
            let path = if parent == "/" {
                sdf::Path::abs_root()
            } else {
                sdf::path(&parent).expect("sanitized prim path")
            };
            let spec = self.data.spec_mut(&path).expect("parent spec exists");
            spec.add(ChildrenKey::PrimChildren, Value::token_vec(names));
        }
        let mut props: Vec<(String, Vec<String>)> = self.prop_children.drain().collect();
        for (prim, names) in props.drain(..) {
            let spec = self
                .data
                .spec_mut(&sdf::path(&prim).expect("sanitized prim path"))
                .expect("prim spec exists");
            spec.add(ChildrenKey::PropertyChildren, Value::token_vec(names));
        }
        let text = TextWriter::write_to_string(&self.data)
            .map_err(|e| UsdExportError::Author(e.to_string()))?;
        Ok(bracket_singleton_api_schemas(&text))
    }
}

enum AttrValue {
    Default(Value),
    Uniform(Value),
    Samples(sdf::TimeSampleMap),
}

enum XformValue<'a> {
    Static(Isometry3<f64>),
    Sampled(&'a [f64], Vec<Isometry3<f64>>),
}

// ----------------------------------------------- USD robot (reference+over)

/// How a body prim's parent transform is reached in the export stage.
enum ParentAnchor {
    /// Below the reference root: `offset` is the static chain from the root
    /// prim's frame to the body's parent prim.
    Root { offset: Isometry3<f64> },
    /// Nested under another animated body (`link` index).
    Body { link: usize, offset: Isometry3<f64> },
}

pub(crate) struct LinkStageInfo {
    /// Path relative to the articulation root; empty when the root prim is
    /// itself the body.
    rel_path: String,
    /// Absolute body prim path in the (possibly re-rooted) stage.
    pub(crate) stage_path: String,
    pub(crate) k_inv: Isometry3<f64>,
    parent: ParentAnchor,
}

pub(crate) struct RobotStageInfo {
    pub(crate) frame: StageFrame,
    root_path: String,
    pub(crate) links: Vec<LinkStageInfo>,
}

/// Prim facts gathered from the source robot stage.
struct StagePrim {
    world: Isometry3<f64>,
    residual_ok: bool,
    type_name: String,
    prim: openusd::usd::Prim,
}

/// A stage opened for structural inspection: frame conventions plus every
/// prim's (earliest-time) world transform and type.
pub(crate) struct OpenedStage {
    pub(crate) stage: Stage,
    pub(crate) frame: StageFrame,
    pub(crate) path: String,
    prims: HashMap<String, StagePrim>,
}

impl OpenedStage {
    pub(crate) fn has_prim(&self, path: &str) -> bool {
        self.prims.contains_key(path)
    }
}

/// Stage normalization (USD defaults: centimeters, Y-up) — must mirror the
/// importer exactly.
fn frame_from_stage(stage: &Stage) -> StageFrame {
    let mut frame = StageFrame {
        mpu: 0.01,
        up_fix: y_up_to_z_up(),
    };
    let layer = stage.root_layer();
    if let Some(root) = layer.pseudo_root() {
        match root.field("metersPerUnit") {
            Ok(Some(Value::Double(d))) => frame.mpu = d,
            Ok(Some(Value::Float(f))) => frame.mpu = f as f64,
            _ => {}
        }
        if let Ok(Some(Value::Token(t))) = root.field("upAxis") {
            if t.as_str() == "Z" {
                frame.up_fix = UnitQuaternion::identity();
            }
        }
    }
    frame
}

/// Frame metadata of a stage without traversing its prims (cheap enough to
/// peek at a robot stage's conventions during recording import).
pub(crate) fn stage_frame_metadata(
    path: &Path,
    extra_search_paths: &[PathBuf],
) -> Result<StageFrame, UsdExportError> {
    let mut search_paths = extra_search_paths.to_vec();
    if let Some(dir) = path.parent() {
        search_paths.push(dir.to_path_buf());
    }
    let stage = Stage::builder()
        .resolver(SearchPathResolver::new(search_paths))
        .open(&path.display().to_string())
        .map_err(|e| UsdExportError::Open {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;
    Ok(frame_from_stage(&stage))
}

pub(crate) fn open_stage_prims(
    path: &Path,
    extra_search_paths: &[PathBuf],
) -> Result<OpenedStage, UsdExportError> {
    let mut search_paths = extra_search_paths.to_vec();
    if let Some(dir) = path.parent() {
        search_paths.push(dir.to_path_buf());
    }
    let stage = Stage::builder()
        .resolver(SearchPathResolver::new(search_paths))
        .open(&path.display().to_string())
        .map_err(|e| UsdExportError::Open {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;

    let frame = frame_from_stage(&stage);

    let mut prims: HashMap<String, StagePrim> = HashMap::new();
    collect_stage_prims(
        stage.prim(sdf::Path::abs_root()),
        gf::Matrix4d::default(),
        &mut prims,
    )
    .map_err(|e| UsdExportError::RobotStage(e.to_string()))?;

    Ok(OpenedStage {
        frame,
        path: path.display().to_string(),
        prims,
        stage,
    })
}

/// Stage prim path of a model-named prim when the robot subtree rooted at
/// `model_root` (the model's articulation root) lives at `stage_root` in
/// this stage. Identity when the two roots coincide.
pub(crate) fn remap_model_path(stage_root: &str, model_root: &str, name: &str) -> String {
    if stage_root == model_root || name == "/" {
        return name.to_string();
    }
    if name == model_root {
        return stage_root.to_string();
    }
    match name.strip_prefix(&format!("{model_root}/")) {
        Some(rel) => format!("{stage_root}/{rel}"),
        None => name.to_string(),
    }
}

/// Finds where the robot's link tree lives in an opened stage: the model's
/// own articulation root when present (an animation layered over the robot
/// stage), otherwise the unique prim whose subtree contains every link
/// (e.g. `/World/Robot` in a botrail export, or wherever a recording
/// placed the robot).
pub(crate) fn find_robot_root(
    opened: &OpenedStage,
    model_root: &str,
    model: &RobotModel,
) -> Result<String, UsdExportError> {
    let all_present = |root: &str| {
        model.links.iter().all(|l| {
            opened
                .prims
                .contains_key(remap_model_path(root, model_root, &l.name).as_str())
        })
    };
    if all_present(model_root) {
        return Ok(model_root.to_string());
    }
    let mut candidates: Vec<String> = opened
        .prims
        .keys()
        .filter(|p| p.as_str() != "/" && all_present(p))
        .cloned()
        .collect();
    candidates.sort();
    match candidates.len() {
        0 => Err(UsdExportError::RobotStage(format!(
            "no prim in `{}` carries this robot's link tree (looked for `{}` and its siblings)",
            opened.path, model.links[model.root_link].name
        ))),
        1 => Ok(candidates.remove(0)),
        _ => Err(UsdExportError::RobotStage(format!(
            "multiple prims in `{}` match this robot's link tree: {}",
            opened.path,
            candidates.join(", ")
        ))),
    }
}

pub(crate) fn robot_stage_info(
    path: &Path,
    extra_search_paths: &[PathBuf],
    articulation_root: &str,
    model: &RobotModel,
    warnings: &mut Vec<String>,
) -> Result<(RobotStageInfo, Stage), UsdExportError> {
    let opened = open_stage_prims(path, extra_search_paths)?;
    let info = robot_stage_info_on(
        &opened,
        articulation_root,
        articulation_root,
        model,
        warnings,
    )?;
    Ok((info, opened.stage))
}

/// Structural facts about the robot inside an opened stage. `stage_root` is
/// the robot's root prim in *this* stage; `model_root` the model's own
/// articulation root (they differ when a recording re-rooted the robot).
pub(crate) fn robot_stage_info_on(
    opened: &OpenedStage,
    stage_root: &str,
    model_root: &str,
    model: &RobotModel,
    warnings: &mut Vec<String>,
) -> Result<RobotStageInfo, UsdExportError> {
    let frame = opened.frame;
    let prims = &opened.prims;
    let articulation_root = stage_root;

    if articulation_root == "/" {
        return Err(UsdExportError::RobotStage(
            "cannot reference a pseudo-root articulation; the robot stage needs a root prim".into(),
        ));
    }
    if !prims.contains_key(articulation_root) {
        return Err(UsdExportError::RobotStage(format!(
            "articulation root `{articulation_root}` not found in `{}`",
            opened.path
        )));
    }

    // Inbound-joint corrections (K = localPose1), keyed by body path —
    // considering exactly the joint types the importer consumed.
    let in_subtree = |p: &str| {
        p == articulation_root
            || p.starts_with(&format!("{articulation_root}/"))
            || articulation_root == "/"
    };
    let mut corrections: HashMap<String, Isometry3<f64>> = HashMap::new();
    for (prim_path, info) in prims {
        if !in_subtree(prim_path) {
            continue;
        }
        if !matches!(
            info.type_name.as_str(),
            "PhysicsRevoluteJoint" | "PhysicsPrismaticJoint" | "PhysicsFixedJoint"
        ) {
            continue;
        }
        let joint = AnyJoint(info.prim.clone());
        let body1 = joint
            .body1_rel()
            .targets()
            .ok()
            .and_then(|t| t.first().map(|p| p.to_string()));
        let Some(body1) = body1 else { continue };
        let read_pos = |attr: openusd::usd::Attribute| -> Vector3<f64> {
            attr.get::<[f32; 3]>()
                .ok()
                .flatten()
                .map(|p| Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64))
                .unwrap_or_else(Vector3::zeros)
        };
        let read_rot = |attr: openusd::usd::Attribute| -> UnitQuaternion<f64> {
            attr.get::<gf::Quatf>()
                .ok()
                .flatten()
                .map(|q| {
                    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        q.w as f64, q.x as f64, q.y as f64, q.z as f64,
                    ))
                })
                .unwrap_or_else(UnitQuaternion::identity)
        };
        let local_pose1 = Isometry3::from_parts(
            Translation3::from(read_pos(joint.local_pos1_attr())),
            read_rot(joint.local_rot1_attr()),
        );
        corrections.insert(body1, local_pose1);
    }

    // Model link names are body prim paths (the importer's naming contract),
    // re-rooted at `stage_root` when the stage placed the robot elsewhere.
    let body_set: HashMap<String, usize> = model
        .links
        .iter()
        .enumerate()
        .map(|(i, l)| (remap_model_path(stage_root, model_root, &l.name), i))
        .collect();
    let mut links = Vec::with_capacity(model.links.len());
    for link in &model.links {
        let body_path = remap_model_path(stage_root, model_root, &link.name);
        let body_path = body_path.as_str();
        let Some(body) = prims.get(body_path) else {
            return Err(UsdExportError::RobotStage(format!(
                "model link `{body_path}` has no prim in `{}` — was the robot imported from this stage?",
                opened.path
            )));
        };
        if !body.residual_ok {
            warnings.push(format!(
                "{body_path}: non-rigid (scaled/sheared) prim transform; exported motion may be off"
            ));
        }
        let rel_path = if body_path == articulation_root {
            String::new()
        } else {
            body_path
                .strip_prefix(&format!("{articulation_root}/"))
                .unwrap_or(body_path)
                .to_string()
        };

        // Walk up to the nearest animated body (or the reference root).
        let parent_path = &body_path[..body_path.rfind('/').unwrap_or(0).max(1)];
        let parent_world = prims
            .get(parent_path)
            .map(|p| p.world)
            .unwrap_or_else(Isometry3::identity);
        let mut anchor_path = parent_path.to_string();
        let anchor = loop {
            if anchor_path == articulation_root || anchor_path.is_empty() || anchor_path == "/" {
                let root_world = prims[articulation_root].world;
                break ParentAnchor::Root {
                    offset: root_world.inverse() * parent_world,
                };
            }
            if let Some(&link_idx) = body_set.get(anchor_path.as_str()) {
                let anchor_world = prims[anchor_path.as_str()].world;
                break ParentAnchor::Body {
                    link: link_idx,
                    offset: anchor_world.inverse() * parent_world,
                };
            }
            anchor_path = anchor_path[..anchor_path.rfind('/').unwrap_or(0).max(1)].to_string();
        };

        links.push(LinkStageInfo {
            rel_path,
            stage_path: body_path.to_string(),
            k_inv: corrections
                .get(body_path)
                .map(|k| k.inverse())
                .unwrap_or_else(Isometry3::identity),
            parent: anchor,
        });
    }

    Ok(RobotStageInfo {
        frame,
        root_path: articulation_root.to_string(),
        links,
    })
}

fn collect_stage_prims(
    prim: openusd::usd::Prim,
    parent_world: gf::Matrix4d,
    out: &mut HashMap<String, StagePrim>,
) -> anyhow::Result<()> {
    let view = AnyPrim(prim);
    let world = view
        .local_to_parent_transform(TimeCode::EARLIEST)
        .map(|local| local * parent_world)
        .unwrap_or(parent_world);
    let path = view.prim().path().to_string();
    let type_name = view
        .prim()
        .type_name()?
        .map(|t| t.to_string())
        .unwrap_or_default();
    let children = view.prim().children()?;
    let (iso, residual) = decompose_matrix(&world);
    let residual_ok = (residual - nalgebra::Matrix3::identity()).norm() < 1e-6;
    out.insert(
        path,
        StagePrim {
            world: iso,
            residual_ok,
            type_name,
            prim: view.0,
        },
    );
    for child in children {
        collect_stage_prims(child, world, out)?;
    }
    Ok(())
}

fn author_referenced_robot(
    layer: &mut LayerBuilder,
    robot: &RobotAnimation,
    codes: &[f64],
    info: &RobotStageInfo,
    robot_prim: &str,
    asset_ref_dir: &str,
    warnings: &mut Vec<String>,
) -> Result<(), UsdExportError> {
    layer.ensure_prim(robot_prim, Specifier::Def, Some("Xform"));
    layer.prim_field(
        robot_prim,
        FieldKey::References,
        Value::ReferenceListOp(ListOp {
            prepended_items: vec![Reference {
                asset_path: format!("{asset_ref_dir}/{}", asset_root_name(robot.model)?),
                prim_path: sdf::path(&info.root_path)
                    .map_err(|e| UsdExportError::Author(e.to_string()))?,
                layer_offset: Default::default(),
                custom_data: HashMap::new(),
            }],
            ..Default::default()
        }),
    );

    // Corrective placement: the referenced subtree stays in source-stage
    // units/orientation; `orient = F, scale = mpu` maps it into the Z-up
    // meter export world (identity for Z-up/meter sources). This also
    // replaces whatever transform the source authored on the root prim.
    let root_is_body = info.links.iter().position(|l| l.rel_path.is_empty());
    let mpu = info.frame.mpu;
    let fix = Isometry3::from_parts(Translation3::identity(), info.frame.up_fix);
    if let Some(root_link) = root_is_body {
        // The root prim is itself a body: fold the corrective into its
        // animated ops (translate/orient carry F and the unit scale).
        let poses: Vec<Isometry3<f64>> = (0..codes.len())
            .map(|f| {
                let body = info.frame.to_stage(&robot.link_poses[f][root_link])
                    * info.links[root_link].k_inv;
                Isometry3::from_parts(
                    Translation3::from(info.frame.up_fix * (body.translation.vector * mpu)),
                    info.frame.up_fix * body.rotation,
                )
            })
            .collect();
        layer.xform(
            robot_prim,
            &XformValue::Sampled(codes, poses),
            Some([mpu, mpu, mpu]),
        );
    } else {
        layer.xform(robot_prim, &XformValue::Static(fix), Some([mpu, mpu, mpu]));
    }

    // Per-frame stage-coordinate body poses, then parent-relative locals.
    let n = codes.len();
    let mut body_stage: Vec<Vec<Isometry3<f64>>> = Vec::with_capacity(n);
    for f in 0..n {
        body_stage.push(
            info.links
                .iter()
                .enumerate()
                .map(|(i, link)| info.frame.to_stage(&robot.link_poses[f][i]) * link.k_inv)
                .collect(),
        );
    }
    for (i, link) in info.links.iter().enumerate() {
        if link.rel_path.is_empty() {
            continue; // authored on the robot prim above
        }
        let prim = format!("{robot_prim}/{}", link.rel_path);
        let poses: Vec<Isometry3<f64>> = (0..n)
            .map(|f| {
                let parent = match &link.parent {
                    ParentAnchor::Root { offset } => *offset,
                    ParentAnchor::Body { link: j, offset } => body_stage[f][*j] * offset,
                };
                parent.inverse() * body_stage[f][i]
            })
            .collect();
        layer.ensure_prim(&prim, Specifier::Over, None);
        layer.xform(&prim, &XformValue::Sampled(codes, poses), None);
    }

    if let Some(joint_samples) = robot.joint_samples {
        author_joint_states(
            layer,
            robot,
            codes,
            joint_samples,
            info,
            robot_prim,
            warnings,
        );
    }
    Ok(())
}

/// Authors `PhysicsJointStateAPI` position timeSamples on the joint prims —
/// the articulation-native encoding of the same animation, letting readers
/// recover q(t) without projecting link poses. Angular positions follow the
/// UsdPhysics convention (degrees); linear ones are in stage units.
fn author_joint_states(
    layer: &mut LayerBuilder,
    robot: &RobotAnimation,
    codes: &[f64],
    joint_samples: &[Vec<f64>],
    info: &RobotStageInfo,
    robot_prim: &str,
    warnings: &mut Vec<String>,
) {
    use botrail_model::JointType;
    let root_prefix = format!("{}/", info.root_path);
    for joint in &robot.model.joints {
        let Some(qi) = joint.q_index else { continue };
        let Some(rel) = joint.name.strip_prefix(&root_prefix) else {
            warnings.push(format!(
                "joint `{}` lies outside the articulation root; joint state not authored",
                joint.name
            ));
            continue;
        };
        let (instance, to_stage): (&str, fn(f64, f64) -> f64) = match joint.joint_type {
            JointType::Revolute | JointType::Continuous => ("angular", |v, _| v.to_degrees()),
            JointType::Prismatic => ("linear", |v, mpu| v / mpu),
            JointType::Fixed => continue,
        };
        let prim = format!("{robot_prim}/{rel}");
        layer.ensure_prim(&prim, Specifier::Over, None);
        layer.prim_field(
            &prim,
            FieldKey::ApiSchemas,
            Value::TokenListOp(ListOp {
                prepended_items: vec![format!("PhysicsJointStateAPI:{instance}").into()],
                ..Default::default()
            }),
        );
        let samples: sdf::TimeSampleMap = codes
            .iter()
            .zip(joint_samples)
            .map(|(code, q)| (*code, Value::Float(to_stage(q[qi], info.frame.mpu) as f32)))
            .collect();
        layer.attr(
            &prim,
            &format!("state:{instance}:physics:position"),
            "float",
            AttrValue::Samples(samples),
        );
    }
}

/// File name of the robot's root stage inside the copied asset directory.
fn asset_root_name(model: &RobotModel) -> Result<String, UsdExportError> {
    match &model.source {
        RobotSource::Usd { path, .. } => path
            .file_name()
            .map(|f| f.to_string_lossy().to_string())
            .ok_or_else(|| UsdExportError::RobotStage("robot stage path has no file name".into())),
        _ => unreachable!("only called for USD robots"),
    }
}

/// Copy plan for the robot stage and its layer dependencies, mirroring the
/// `.botrail` bundling scheme (paths relative to the stage directory).
/// `dir_name` is the per-source directory under `<stem>_assets/`.
fn robot_asset_copies(
    stage_path: &Path,
    asset_stem: &str,
    dir_name: &str,
    warnings: &mut Vec<String>,
) -> Result<Vec<(PathBuf, PathBuf)>, UsdExportError> {
    let deps = crate::stage_dependencies(stage_path, &[])
        .map_err(|e| UsdExportError::RobotStage(e.to_string()))?;
    let stage_dir = stage_path.parent().unwrap_or(Path::new(""));
    let mut copies = Vec::new();
    for dep in deps {
        match dep.strip_prefix(stage_dir) {
            Ok(rel) => copies.push((
                dep.clone(),
                Path::new(&format!("{asset_stem}_assets/{dir_name}")).join(rel),
            )),
            Err(_) => warnings.push(format!(
                "robot layer `{}` lies outside the stage directory; not copied (the reference may not resolve elsewhere)",
                dep.display()
            )),
        }
    }
    Ok(copies)
}

// -------------------------------------------------- URDF robot (authored)

fn author_urdf_robot(
    layer: &mut LayerBuilder,
    robot: &RobotAnimation,
    codes: &[f64],
    robot_prim: &str,
    warnings: &mut Vec<String>,
) -> Result<(), UsdExportError> {
    layer.ensure_prim(robot_prim, Specifier::Def, Some("Xform"));
    let mut used = HashMap::new();
    for (i, link) in robot.model.links.iter().enumerate() {
        let name = unique_child(&mut used, &sanitize_name(&link.name));
        let prim = format!("{robot_prim}/{name}");
        layer.ensure_prim(&prim, Specifier::Def, Some("Xform"));
        let poses: Vec<Isometry3<f64>> = robot.link_poses.iter().map(|p| p[i]).collect();
        layer.xform(&prim, &XformValue::Sampled(codes, poses), None);
        for (vi, shape) in link.visuals.iter().enumerate() {
            let shape_prim = format!("{prim}/Visual_{vi}");
            author_shape(layer, &shape_prim, shape, warnings)?;
        }
    }
    Ok(())
}

/// Authors one visual shape as a gprim with a static link-local transform.
fn author_shape(
    layer: &mut LayerBuilder,
    prim: &str,
    shape: &Shape,
    warnings: &mut Vec<String>,
) -> Result<(), UsdExportError> {
    author_geometry(
        layer,
        prim,
        &shape.geometry,
        &XformValue::Static(shape.origin),
        None,
        warnings,
    )
}

/// Fallback shade for geometry with no authored colour of its own.
const ENV_COLOR: [f32; 3] = [0.604, 0.639, 0.698];

/// Authors a geometry prim (with display color) whose transform is `pose`.
fn author_geometry(
    layer: &mut LayerBuilder,
    prim: &str,
    geometry: &Geometry,
    pose: &XformValue,
    color: Option<[f32; 3]>,
    _warnings: &mut Vec<String>,
) -> Result<(), UsdExportError> {
    let extent = |half: [f64; 3]| {
        Value::Vec3fVec(vec![
            gf::vec3f(-half[0] as f32, -half[1] as f32, -half[2] as f32),
            gf::vec3f(half[0] as f32, half[1] as f32, half[2] as f32),
        ])
    };
    match geometry {
        Geometry::Box { size } => {
            layer.ensure_prim(prim, Specifier::Def, Some("Cube"));
            layer.attr(
                prim,
                "size",
                "double",
                AttrValue::Default(Value::Double(1.0)),
            );
            layer.attr(
                prim,
                "extent",
                "float3[]",
                AttrValue::Default(extent([0.5, 0.5, 0.5])),
            );
            layer.xform(prim, pose, Some([size.x, size.y, size.z]));
        }
        Geometry::Sphere { radius } => {
            layer.ensure_prim(prim, Specifier::Def, Some("Sphere"));
            layer.attr(
                prim,
                "radius",
                "double",
                AttrValue::Default(Value::Double(*radius)),
            );
            layer.attr(
                prim,
                "extent",
                "float3[]",
                AttrValue::Default(extent([*radius, *radius, *radius])),
            );
            layer.xform(prim, pose, None);
        }
        Geometry::Cylinder { radius, length } => {
            // UsdGeomCylinder's default axis is Z — same as URDF/botrail.
            layer.ensure_prim(prim, Specifier::Def, Some("Cylinder"));
            layer.attr(
                prim,
                "radius",
                "double",
                AttrValue::Default(Value::Double(*radius)),
            );
            layer.attr(
                prim,
                "height",
                "double",
                AttrValue::Default(Value::Double(*length)),
            );
            layer.attr(
                prim,
                "extent",
                "float3[]",
                AttrValue::Default(extent([*radius, *radius, length / 2.0])),
            );
            layer.xform(prim, pose, None);
        }
        Geometry::Mesh { path, scale } => {
            let data = botrail_mesh::load_path(path).map_err(|e| UsdExportError::Mesh {
                path: path.display().to_string(),
                message: e.to_string(),
            })?;
            let points: Vec<gf::Vec3f> = data
                .vertices
                .iter()
                .map(|v| {
                    gf::vec3f(
                        (v[0] * scale.x) as f32,
                        (v[1] * scale.y) as f32,
                        (v[2] * scale.z) as f32,
                    )
                })
                .collect();
            let (mut min, mut max) = ([f32::MAX; 3], [f32::MIN; 3]);
            for p in &points {
                for (k, v) in [p.x, p.y, p.z].into_iter().enumerate() {
                    min[k] = min[k].min(v);
                    max[k] = max[k].max(v);
                }
            }
            let indices: Vec<i32> = data
                .indices
                .iter()
                .flat_map(|t| t.iter().map(|&i| i as i32))
                .collect();
            let counts = vec![3i32; data.indices.len()];
            layer.ensure_prim(prim, Specifier::Def, Some("Mesh"));
            layer.attr(
                prim,
                "points",
                "point3f[]",
                AttrValue::Default(Value::Vec3fVec(points)),
            );
            layer.attr(
                prim,
                "faceVertexCounts",
                "int[]",
                AttrValue::Default(Value::IntVec(counts)),
            );
            layer.attr(
                prim,
                "faceVertexIndices",
                "int[]",
                AttrValue::Default(Value::IntVec(indices)),
            );
            layer.attr(
                prim,
                "extent",
                "float3[]",
                AttrValue::Default(Value::Vec3fVec(vec![
                    gf::vec3f(min[0], min[1], min[2]),
                    gf::vec3f(max[0], max[1], max[2]),
                ])),
            );
            layer.attr(
                prim,
                "subdivisionScheme",
                "token",
                AttrValue::Uniform(Value::Token(tf::Token::from("none"))),
            );
            layer.xform(prim, pose, None);
        }
    }
    let [r, g, b] = color.unwrap_or(ENV_COLOR);
    layer.attr(
        prim,
        "primvars:displayColor",
        "color3f[]",
        AttrValue::Default(Value::Vec3fVec(vec![gf::vec3f(r, g, b)])),
    );
    Ok(())
}

fn unique_child(used: &mut HashMap<String, usize>, name: &str) -> String {
    let count = used.entry(name.to_string()).or_insert(0);
    *count += 1;
    if *count == 1 {
        name.to_string()
    } else {
        format!("{name}_{count}")
    }
}

// ------------------------------------------------------------- environment

fn author_objects(
    layer: &mut LayerBuilder,
    objects: &[ObjectSpec],
    codes: &[f64],
    warnings: &mut Vec<String>,
) -> Result<(), UsdExportError> {
    if objects.is_empty() {
        return Ok(());
    }
    layer.ensure_prim("/World/Env", Specifier::Def, Some("Xform"));
    for obj in objects {
        let mut parent = "/World/Env".to_string();
        let segments: Vec<&str> = obj.name.split('/').filter(|s| !s.is_empty()).collect();
        let (leaf, mids) = segments.split_last().unwrap_or((&"object", &[]));
        // Intermediate segments are shared grouping Xforms (idempotent);
        // only the leaf gprim must not collide with an existing prim.
        for mid in mids {
            let path = format!("{parent}/{}", sanitize_name(mid));
            layer.ensure_prim(&path, Specifier::Def, Some("Xform"));
            parent = path;
        }
        let base = sanitize_name(leaf);
        let mut name = base.clone();
        let mut n = 1;
        while layer.has_prim(&format!("{parent}/{name}")) {
            n += 1;
            name = format!("{base}_{n}");
        }
        let prim = format!("{parent}/{name}");
        let pose = match &obj.track {
            PoseTrack::Static(x) => XformValue::Static(*x),
            PoseTrack::Sampled(samples) => XformValue::Sampled(codes, samples.clone()),
        };
        author_geometry(layer, &prim, &obj.geometry, &pose, obj.color, warnings)?;
        if !obj.visible.is_empty() {
            let samples: sdf::TimeSampleMap = codes
                .iter()
                .enumerate()
                .map(|(k, &code)| {
                    let shown = obj.visible.get(k).copied().unwrap_or(true);
                    let token = if shown { "inherited" } else { "invisible" };
                    (code, Value::Token(tf::Token::from(token)))
                })
                .collect();
            layer.attr(&prim, "visibility", "token", AttrValue::Samples(samples));
        }
    }
    Ok(())
}

/// Shared 2-DOF test articulation (export + recording tests).
/// A 2-DOF articulation exercising every export-relevant feature: a
/// nontrivial K on link1 (localPos1 = (0,0,-0.2)), and link2 nested
/// under a *static intermediate* Xform below link1 (anchor = body
/// ancestor + folded static offset).
#[cfg(test)]
pub(crate) const TEST_ARM: &str = r#"#usda 1.0
(
    defaultPrim = "Robot"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Robot" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom"
        {
            double size = 0.1
        }
    }

    def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom"
        {
            double size = 0.1
        }

        def Xform "carriage"
        {
            double3 xformOp:translate = (0.05, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]

            def Xform "link2" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
            {
                def Cube "geom"
                {
                    double size = 0.1
                }
            }
        }
    }

    def Scope "joints"
    {
        def PhysicsFixedJoint "anchor"
        {
            rel physics:body1 = </Robot/base>
        }

        def PhysicsRevoluteJoint "j1"
        {
            rel physics:body0 = </Robot/base>
            rel physics:body1 = </Robot/link1>
            uniform token physics:axis = "Z"
            point3f physics:localPos0 = (0, 0, 0.5)
            point3f physics:localPos1 = (0, 0, -0.2)
            float physics:lowerLimit = -90
            float physics:upperLimit = 90
        }

        def PhysicsRevoluteJoint "j2"
        {
            rel physics:body0 = </Robot/link1>
            rel physics:body1 = </Robot/link1/carriage/link2>
            uniform token physics:axis = "Y"
            point3f physics:localPos0 = (0, 0, 0.2)
            float physics:lowerLimit = -120
            float physics:upperLimit = 120
        }
    }
}
"#;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::articulation::{import_robot, RobotImportOptions};
    use nalgebra::Matrix3;

    const ARM: &str = TEST_ARM;

    /// The same layer reinterpreted as a centimeters / Y-up stage — a
    /// *different* robot, but import→FK vs export→compose must still agree.
    fn arm_yup_cm() -> String {
        ARM.replace("metersPerUnit = 1", "metersPerUnit = 0.01")
            .replace("upAxis = \"Z\"", "upAxis = \"Y\"")
    }

    fn temp_dir(tag: &str) -> PathBuf {
        static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "botrail-usd-export-{tag}-{}-{n}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Row-vector point transform: `p' = [p, 1] · m`.
    fn mul_point(m: &gf::Matrix4d, p: [f64; 3]) -> [f64; 3] {
        let v = &m.0;
        [
            p[0] * v[0] + p[1] * v[4] + p[2] * v[8] + v[12],
            p[0] * v[1] + p[1] * v[5] + p[2] * v[9] + v[13],
            p[0] * v[2] + p[1] * v[6] + p[2] * v[10] + v[14],
        ]
    }

    /// Composed world matrices of every prim at `code`.
    fn composed_worlds(stage: &Stage, code: f64) -> HashMap<String, gf::Matrix4d> {
        fn walk(
            prim: openusd::usd::Prim,
            parent: gf::Matrix4d,
            code: f64,
            out: &mut HashMap<String, gf::Matrix4d>,
        ) {
            let view = AnyPrim(prim);
            let world = view
                .local_to_parent_transform(TimeCode::new(code))
                .map(|local| local * parent)
                .unwrap_or(parent);
            out.insert(view.prim().path().to_string(), world);
            for child in view.prim().children().unwrap_or_default() {
                walk(child, world, code, out);
            }
        }
        let mut out = HashMap::new();
        walk(
            stage.prim(sdf::Path::abs_root()),
            gf::Matrix4d::default(),
            code,
            &mut out,
        );
        out
    }

    fn assert_close(a: [f64; 3], b: [f64; 3], tol: f64, ctx: &str) {
        let d = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
        assert!(d < tol, "{ctx}: {a:?} vs {b:?} (d = {d})");
    }

    /// Import a robot stage, bake an FK animation, export with reference +
    /// overs, recompose the exported stage, and check that every link's
    /// *geometry* lands where botrail FK puts it — mapping the raw cube
    /// center and a corner through both pipelines (basis-independent).
    fn reference_roundtrip(usda: &str, tag: &str) {
        let dir = temp_dir(tag);
        std::fs::write(dir.join("robot.usda"), usda).unwrap();
        let imported = import_robot(
            &dir.join("robot.usda"),
            &RobotImportOptions {
                mesh_cache_dir: Some(dir.join("meshes")),
                ..Default::default()
            },
        )
        .unwrap();
        let model = imported.model;

        let base = Isometry3::from_parts(
            Translation3::new(0.3, -0.2, 0.1),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.4),
        );
        let times = [0.0, 0.5, 1.25];
        let configs = [vec![0.0, 0.0], vec![0.7, -0.4], vec![-1.2, 1.8]];
        let link_poses: Vec<Vec<Isometry3<f64>>> = configs
            .iter()
            .map(|q| botrail_kin::forward_kinematics_with_base(&model, q, &base).unwrap())
            .collect();

        let joint_samples: Vec<Vec<f64>> = configs.to_vec();
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &link_poses,
            joint_samples: Some(&joint_samples),
        }];
        let input = AnimationInput {
            robots: &robots,
            times: &times,
            objects: &[],
        };
        let warnings =
            write_animation(&dir.join("anim.usda"), &input, &ExportOptions::default()).unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");

        // Raw half-size of the fixture cubes (authored size 0.1).
        let half_raw = 0.05;
        // Stage normalization of the source (to map raw points to botrail).
        let source_is_zup = usda.contains("upAxis = \"Z\"");
        let mpu = if usda.contains("metersPerUnit = 1") {
            1.0
        } else {
            0.01
        };
        let up_fix = if source_is_zup {
            UnitQuaternion::identity()
        } else {
            y_up_to_z_up()
        };
        let to_botrail =
            |p: [f64; 3]| -> Vector3<f64> { up_fix * (Vector3::new(p[0], p[1], p[2]) * mpu) };

        let stage = Stage::builder()
            .resolver(SearchPathResolver::new(vec![dir.clone()]))
            .open(&dir.join("anim.usda").display().to_string())
            .unwrap();

        for (f, t) in times.iter().enumerate() {
            let worlds = composed_worlds(&stage, t * 60.0);
            for (i, link) in model.links.iter().enumerate() {
                let rel = link.name.strip_prefix("/Robot").unwrap();
                let geom_path = format!("/World/Robot{rel}/geom");
                let m = worlds
                    .get(&geom_path)
                    .unwrap_or_else(|| panic!("missing {geom_path}"));
                // Botrail side: FK link pose ∘ imported visual origin,
                // applied to the baked-coordinate cube points.
                let visual = link_poses[f][i] * link.visuals[0].origin;
                for p_raw in [[0.0, 0.0, 0.0], [half_raw, half_raw, half_raw]] {
                    let via_export = mul_point(m, p_raw);
                    let expected = visual * nalgebra::Point3::from(to_botrail(p_raw));
                    assert_close(
                        via_export,
                        [expected.x, expected.y, expected.z],
                        1e-9,
                        &format!("{tag} frame {f} link {} point {p_raw:?}", link.name),
                    );
                }
            }
        }

        // pxr requires apiSchemas as a bracketed list even for one entry
        // (openusd's scalar shorthand is rejected by stock USD tooling).
        let text = std::fs::read_to_string(dir.join("anim.usda")).unwrap();
        assert!(
            text.contains("prepend apiSchemas = [\"PhysicsJointStateAPI:angular\"]"),
            "apiSchemas not bracketed"
        );

        // JointState timeSamples carry q(t) natively (angular in degrees).
        for (f, t) in times.iter().enumerate() {
            for (ji, joint) in model.joints.iter().enumerate() {
                let Some(qi) = joint.q_index else { continue };
                let rel = joint.name.strip_prefix("/Robot").unwrap();
                let raw = stage
                    .attribute(
                        sdf::path(format!("/World/Robot{rel}"))
                            .unwrap()
                            .append_property("state:angular:physics:position")
                            .unwrap(),
                    )
                    .get_at::<sdf::Value>(TimeCode::new(t * 60.0))
                    .unwrap()
                    .unwrap_or_else(|| panic!("missing joint state on {}", joint.name));
                // The text parser may widen `float` samples to Double.
                let attr = match raw {
                    sdf::Value::Float(v) => v as f64,
                    sdf::Value::Double(v) => v,
                    other => panic!("unexpected joint state value {other:?}"),
                };
                let expected = configs[f][qi].to_degrees();
                assert!(
                    (attr - expected).abs() < 1e-3,
                    "{tag} frame {f} joint {ji}: {attr} vs {expected}"
                );
            }
        }
    }

    #[test]
    fn zup_meter_reference_roundtrip() {
        reference_roundtrip(ARM, "zup");
    }

    #[test]
    fn yup_cm_reference_roundtrip() {
        reference_roundtrip(&arm_yup_cm(), "yup");
    }

    #[test]
    fn urdf_export_is_self_contained() {
        let dir = temp_dir("urdf");
        let stl = dir.join("blade.stl");
        std::fs::write(
            &stl,
            botrail_mesh::to_stl_binary(&botrail_mesh::box_mesh([0.1, 0.2, 0.3])),
        )
        .unwrap();
        let urdf = format!(
            r#"
        <robot name="r">
          <link name="base link">
            <visual><geometry><box size="0.2 0.1 0.4"/></geometry></visual>
            <visual><origin xyz="0 0 0.3"/><geometry><cylinder radius="0.05" length="0.2"/></geometry></visual>
          </link>
          <link name="tip">
            <visual><geometry><mesh filename="{}"/></geometry></visual>
          </link>
          <joint name="j" type="revolute">
            <parent link="base link"/><child link="tip"/>
            <origin xyz="0 0 0.5"/>
            <axis xyz="0 0 1"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
        </robot>"#,
            stl.display()
        );
        let model = RobotModel::from_urdf_str(&urdf).unwrap();

        let times = [0.0, 1.0];
        let configs = [vec![0.0], vec![0.9]];
        let link_poses: Vec<Vec<Isometry3<f64>>> = configs
            .iter()
            .map(|q| botrail_kin::forward_kinematics(&model, q).unwrap())
            .collect();
        let held_track = vec![
            Isometry3::translation(0.1, 0.0, 0.5),
            Isometry3::translation(0.2, 0.1, 0.6),
        ];
        let objects = vec![
            ObjectSpec {
                name: "/World/Conveyor/Box_A".into(),
                geometry: Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                track: PoseTrack::Static(Isometry3::translation(1.0, 0.0, 0.05)),
                color: Some([0.8, 0.3, 0.1]),
                visible: Vec::new(),
            },
            ObjectSpec {
                name: "held".into(),
                geometry: Geometry::Sphere { radius: 0.03 },
                track: PoseTrack::Sampled(held_track.clone()),
                color: None,
                visible: Vec::new(),
            },
        ];
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &link_poses,
            joint_samples: None,
        }];
        let input = AnimationInput {
            robots: &robots,
            times: &times,
            objects: &objects,
        };
        let warnings =
            write_animation(&dir.join("anim.usda"), &input, &ExportOptions::default()).unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");
        // Self-contained: no referenced assets.
        assert!(!dir.join("anim_assets").exists());

        let stage = Stage::builder()
            .resolver(SearchPathResolver::new(vec![dir.clone()]))
            .open(&dir.join("anim.usda").display().to_string())
            .unwrap();

        for (f, t) in times.iter().enumerate() {
            let worlds = composed_worlds(&stage, t * 60.0);
            // Sanitized link prim follows FK (identity parents → world).
            let m = worlds
                .get("/World/Robot/base_link")
                .expect("base link prim");
            let expect = link_poses[f][0].translation;
            assert_close(
                mul_point(m, [0.0; 3]),
                [expect.x, expect.y, expect.z],
                1e-9,
                &format!("base frame {f}"),
            );
            let m = worlds.get("/World/Robot/tip").expect("tip prim");
            let expect = link_poses[f][1].translation;
            assert_close(
                mul_point(m, [0.0; 3]),
                [expect.x, expect.y, expect.z],
                1e-9,
                &format!("tip frame {f}"),
            );
            // The grasped sphere follows its sampled track...
            let m = worlds.get("/World/Env/held").expect("held prim");
            let expect = held_track[f].translation;
            assert_close(
                mul_point(m, [0.0; 3]),
                [expect.x, expect.y, expect.z],
                1e-9,
                &format!("held frame {f}"),
            );
            // ...while the static box stays put, under its nested path.
            let m = worlds
                .get("/World/Env/World/Conveyor/Box_A")
                .expect("static box prim");
            assert_close(
                mul_point(m, [0.0; 3]),
                [1.0, 0.0, 0.05],
                1e-12,
                &format!("box frame {f}"),
            );
        }

        // Each object keeps its own colour; one with none falls back to the
        // neutral environment grey rather than inheriting its neighbour's.
        let color = |path: &str| -> Vec<gf::Vec3f> {
            stage
                .prim(path)
                .attribute("primvars:displayColor")
                .get()
                .unwrap()
                .expect("authored displayColor")
        };
        let painted = color("/World/Env/World/Conveyor/Box_A");
        assert_eq!(painted.len(), 1);
        assert!((painted[0].x - 0.8).abs() < 1e-6, "{:?}", painted[0]);
        assert!((painted[0].y - 0.3).abs() < 1e-6, "{:?}", painted[0]);
        assert!((painted[0].z - 0.1).abs() < 1e-6, "{:?}", painted[0]);
        let fallback = color("/World/Env/held");
        assert!(
            (fallback[0].x - ENV_COLOR[0]).abs() < 1e-6,
            "{:?}",
            fallback[0]
        );

        // The mesh visual made it through with its triangles.
        let usda = std::fs::read_to_string(dir.join("anim.usda")).unwrap();
        assert!(usda.contains("faceVertexIndices"));
        assert!(usda.contains("timeSamples"));
        assert!(usda.contains("upAxis = \"Z\""));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn export_input_validation() {
        let model =
            RobotModel::from_urdf_str(r#"<robot name="r"><link name="only"/></robot>"#).unwrap();
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &[],
            joint_samples: None,
        }];
        let empty = AnimationInput {
            robots: &robots,
            times: &[],
            objects: &[],
        };
        assert!(matches!(
            export_animation(&empty, &ExportOptions::default(), "a"),
            Err(UsdExportError::Input(_))
        ));

        let times = [0.0];
        let bad_len = AnimationInput {
            robots: &robots,
            times: &times,
            objects: &[],
        };
        assert!(matches!(
            export_animation(&bad_len, &ExportOptions::default(), "a"),
            Err(UsdExportError::Input(_))
        ));

        let no_robots = AnimationInput {
            robots: &[],
            times: &times,
            objects: &[],
        };
        assert!(matches!(
            export_animation(&no_robots, &ExportOptions::default(), "a"),
            Err(UsdExportError::Input(_))
        ));
    }

    /// Two instances of one USD asset: each lands under its own
    /// `/World/<name>` with independent motion, the copied asset directory
    /// is shared (dedup), and both references point at it.
    #[test]
    fn two_robots_share_one_asset_copy() {
        let dir = temp_dir("dual");
        std::fs::write(dir.join("robot.usda"), ARM).unwrap();
        let imported = import_robot(
            &dir.join("robot.usda"),
            &RobotImportOptions {
                mesh_cache_dir: Some(dir.join("meshes")),
                ..Default::default()
            },
        )
        .unwrap();
        let model = imported.model;

        let times = [0.0, 0.5];
        let base_a = Isometry3::translation(0.0, -0.4, 0.0);
        let base_b = Isometry3::translation(0.0, 0.4, 0.0);
        let qs_a = [vec![0.0, 0.0], vec![0.8, -0.3]];
        let qs_b = [vec![0.0, 0.0], vec![-0.5, 0.9]];
        let poses = |base: &Isometry3<f64>, qs: &[Vec<f64>]| -> Vec<Vec<Isometry3<f64>>> {
            qs.iter()
                .map(|q| botrail_kin::forward_kinematics_with_base(&model, q, base).unwrap())
                .collect()
        };
        let (poses_a, poses_b) = (poses(&base_a, &qs_a), poses(&base_b, &qs_b));
        let robots = [
            RobotAnimation {
                name: "arm_a",
                model: &model,
                link_poses: &poses_a,
                joint_samples: Some(&qs_a),
            },
            RobotAnimation {
                name: "arm_b",
                model: &model,
                link_poses: &poses_b,
                joint_samples: Some(&qs_b),
            },
        ];
        let input = AnimationInput {
            robots: &robots,
            times: &times,
            objects: &[],
        };
        let warnings =
            write_animation(&dir.join("cell.usda"), &input, &ExportOptions::default()).unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");

        // One shared asset copy, named after the first instance.
        assert!(dir.join("cell_assets/arm_a/robot.usda").exists());
        assert!(!dir.join("cell_assets/arm_b").exists());
        let text = std::fs::read_to_string(dir.join("cell.usda")).unwrap();
        assert_eq!(text.matches("./cell_assets/arm_a/robot.usda").count(), 2);

        // Both robots' geometry composes at their own animated poses.
        let stage = Stage::builder()
            .resolver(SearchPathResolver::new(vec![dir.clone()]))
            .open(&dir.join("cell.usda").display().to_string())
            .unwrap();
        let link1 = model
            .links
            .iter()
            .position(|l| l.name == "/Robot/link1")
            .unwrap();
        for (f, t) in times.iter().enumerate() {
            let worlds = composed_worlds(&stage, t * 60.0);
            for (prim, poses) in [("arm_a", &poses_a), ("arm_b", &poses_b)] {
                let geom = format!("/World/{prim}/link1/geom");
                let m = worlds
                    .get(&geom)
                    .unwrap_or_else(|| panic!("missing {geom}"));
                let expect = (poses[f][link1] * model.links[link1].visuals[0].origin).translation;
                assert_close(
                    mul_point(m, [0.0; 3]),
                    [expect.x, expect.y, expect.z],
                    1e-9,
                    &format!("{prim} frame {f}"),
                );
            }
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn names_are_sanitized() {
        assert_eq!(sanitize_name("panda_link0"), "panda_link0");
        assert_eq!(sanitize_name("base link"), "base_link");
        assert_eq!(sanitize_name("2box"), "_2box");
        assert_eq!(sanitize_name("a-b.c"), "a_b_c");
        assert_eq!(sanitize_name(""), "_");
    }

    #[test]
    fn residual_scale_warns() {
        // Sanity for the residual detector used by the stage walk.
        let m = Matrix3::identity() * 2.0;
        assert!((m - Matrix3::identity()).norm() > 1e-6);
    }
}
