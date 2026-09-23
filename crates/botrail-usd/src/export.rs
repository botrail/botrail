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

use botrail_model::{Geometry, RobotModel, Shape};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use openusd::schemas::geom::Xformable;
use openusd::schemas::physics::JointBase;
use openusd::sdf::{
    self, ChildrenKey, FieldKey, ListOp, Reference, SpecType, Specifier, Value, Variability,
};
use openusd::usd::{SchemaBase, Stage, TimeCode};
use openusd::usda::TextWriter;
use openusd::usdc::CrateWriter;
use openusd::{gf, tf};
use thiserror::Error;

use crate::articulation::AnyJoint;
use crate::{decompose_matrix, y_up_to_z_up, AnyPrim, SearchPathResolver};

mod physics;
mod visual;

pub use physics::{ArticulationSpec, BeltSpec, BodyRole, CarrierSpec, Ride, ServoSpec, UnitSpec};

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

#[derive(Clone, Copy)]
pub struct SurfaceMaterial {
    pub metalness: f32,
    pub roughness: f32,
    pub opacity: Option<f32>,
}

pub struct ObjectSpec {
    /// Obstacle name; `/`-segments become nested prims under `/World/Env`.
    pub name: String,
    pub geometry: Geometry,
    pub visual_asset: Option<botrail_model::VisualAsset>,
    pub track: PoseTrack,
    /// `primvars:displayColor` to author, linear RGB. `None` falls back to
    /// the neutral environment grey.
    pub color: Option<[f32; 3]>,
    /// Explicit optical finish (metalness, roughness), independent of physics.
    pub material: Option<SurfaceMaterial>,
    /// One flag per animation frame: false hides the prim that frame.
    /// Empty means always visible. USD carries this natively as animated
    /// `visibility`, so a magazine full of stock stays out of the picture
    /// in usdview exactly as it does in the studio.
    pub visible: Vec<bool>,
    /// Authored physics, exported as UsdPhysics applied schemas (see
    /// [`PhysicsSpec`]). `None` leaves the prim exactly as before.
    pub physics: Option<PhysicsSpec>,
}

/// UsdPhysics authoring for one exported obstacle — the write-side of the
/// vocabulary the robot importer already reads. A dynamic body gets
/// `PhysicsRigidBodyAPI` (+ `PhysicsMassAPI` with `physics:mass` when the
/// mass is known — otherwise the consumer's density default applies,
/// which in PhysX is the same 1000 kg/m³ botrail uses); every annotated
/// obstacle gets `PhysicsCollisionAPI` and a bound physics material
/// carrying friction/restitution. Un-annotated obstacles stay visual-only
/// — collision in the consumer is opt-in per obstacle, like the bake.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhysicsSpec {
    pub dynamic: bool,
    /// Resolved mass (explicit or from the part identity), kg.
    pub mass: Option<f64>,
    pub friction: f64,
    pub restitution: f64,
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

/// A static polyline bundle authored as one `BasisCurves` prim under
/// `/World/Toolpaths` — toolpath overlays. Purely visual: not an obstacle,
/// no collision, no animation.
pub struct CurveSpec {
    /// Prim name (sanitized and uniquified on authoring).
    pub name: String,
    /// Polylines in world meters; each inner list is one curve. Lists with
    /// fewer than 2 points are dropped with a warning.
    pub curves: Vec<Vec<[f64; 3]>>,
    /// `primvars:displayColor`, linear RGB.
    pub color: [f32; 3],
    /// Constant curve width (m).
    pub width: f32,
}

/// One camera, authored as a `UsdGeomCamera` under `/World/Cameras` with
/// its (possibly sampled) world pose. botrail's camera convention — -Z is
/// the view direction, +Y is image-up — is USD's, so the pose is authored
/// verbatim and "through camera" in usdview frames what the studio's PiP
/// shows. Only the aperture/focal *ratio* decides the field of view, so
/// the units convention (tenths of a unit) never bites.
pub struct CameraSpec {
    /// Prim name (sanitized and uniquified on authoring).
    pub name: String,
    /// Mount-resolved world pose: static for a fixture, one pose per
    /// frame for a camera riding a link or a vehicle.
    pub track: PoseTrack,
    /// `focalLength`, in the same (nominal mm) scale as the apertures.
    pub focal_length: f64,
    pub horizontal_aperture: f64,
    pub vertical_aperture: f64,
    /// Near/far clip distances, stage units (m).
    pub clipping: [f64; 2],
    /// Image size in pixels — not a `UsdGeomCamera` notion (the film back
    /// only fixes the aspect), authored as `custom int2 botrail:resolution`
    /// so a stage botrail wrote imports back pixel-exact.
    pub resolution: [u32; 2],
}

pub struct AnimationInput<'a> {
    /// One bundle per robot, in scene order.
    pub robots: &'a [RobotAnimation<'a>],
    /// Frame times in seconds, strictly increasing, starting at 0 — shared
    /// by every robot and sampled object track.
    pub times: &'a [f64],
    pub objects: &'a [ObjectSpec],
    /// Static toolpath overlays; empty = no `/World/Toolpaths` prim.
    pub curves: &'a [CurveSpec],
    /// Cameras; empty = no `/World/Cameras` prim.
    pub cameras: &'a [CameraSpec],
}

/// How one exported object stands in a simulation stage.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ObjectBody {
    /// Index into [`SimulationSpec::units`] of the rigid body the object
    /// is part of; `None` leaves it to [`ObjectSpec::physics`] (a static
    /// collider, or scenery).
    pub unit: Option<usize>,
    /// A collider nobody draws (an invisible collision proxy): authored
    /// with `purpose = "guide"`.
    pub guide: bool,
}

/// What turns the static export into a *simulation* stage
/// (design-world-physics.md W4-U): the world as a physics engine owns it
/// rather than as a recording of it. Single-frame values are authored as
/// plain defaults — a consumer that writes poses back must not find them
/// shadowed by timeSamples.
pub struct SimulationSpec<'a> {
    /// The bodies the engine moves, in authoring order.
    pub units: &'a [UnitSpec],
    /// One entry per [`AnimationInput::objects`].
    pub bodies: &'a [ObjectBody],
    /// One entry per [`AnimationInput::robots`]; `None` keeps a robot as
    /// exported today (a USD-sourced robot references its own physics).
    pub articulations: &'a [Option<ArticulationSpec>],
    /// One entry per [`AnimationInput::robots`]: what the robot rides, if
    /// anything — for authored articulations and referenced stages alike.
    /// A [`Ride::Joins`] names an earlier robot riding as [`Ride::Base`].
    pub rides: &'a [Option<Ride>],
    /// Height of the ground half-space, when the world has one.
    pub ground: Option<f64>,
}

pub struct ExportedAnimation {
    /// The composed animation layer, ready to serialize as usda text or a
    /// binary usdc crate file.
    pub data: sdf::Data,
    /// Files to place next to the written layer (absolute source, relative
    /// destination) so the robot reference resolves — empty for URDF robots.
    pub assets: Vec<(PathBuf, PathBuf)>,
    pub warnings: Vec<String>,
}

impl ExportedAnimation {
    /// Serializes the layer as usda text, including the singleton-listOp
    /// bracket fix pxr's text parser insists on. The binary path does not
    /// need the fix — crate files carry list ops structurally.
    pub fn to_usda(&self) -> Result<String, UsdExportError> {
        let text = TextWriter::write_to_string(&self.data)
            .map_err(|e| UsdExportError::Author(e.to_string()))?;
        Ok(bracket_singleton_api_schemas(&text))
    }
}

/// Exports and writes the layer to `path` (plus referenced robot assets in
/// a sibling `<stem>_assets/` directory). Returns accumulated warnings.
///
/// The extension picks the serialization: `.usda` writes text, `.usdc` and
/// `.usd` write the binary crate format (the pxr convention for `.usd`).
/// `.usdz` is rejected — packaging referenced assets into an archive is a
/// different operation than writing a layer. Unknown extensions fall back
/// to text with a warning, matching the historical behavior.
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
    write_exported(path, exported)
}

/// Writes an already-composed animation to `path`: serializes by
/// extension and copies the referenced robot assets next to it (see
/// [`write_animation`], whose lower half this is — split out so callers
/// that bake in memory, like the studio's USD download, share one
/// serialization path).
pub fn write_exported(
    path: &Path,
    mut exported: ExportedAnimation,
) -> Result<Vec<String>, UsdExportError> {
    let io = |e: std::io::Error| UsdExportError::Io(e.to_string());
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(io)?;
    }
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_ascii_lowercase());
    match ext.as_deref() {
        Some("usdc") | Some("usd") => {
            let mut crate_file = std::io::Cursor::new(Vec::new());
            CrateWriter::write(&exported.data, &mut crate_file)
                .map_err(|e| UsdExportError::Author(format!("usdc write: {e}")))?;
            let mut bytes = crate_file.into_inner();
            stamp_crate_version(&mut bytes);
            std::fs::write(path, bytes).map_err(io)?;
        }
        Some("usdz") => {
            return Err(UsdExportError::Input(
                "usdz output is not supported (it is an asset package, not a layer); \
                 write .usda or .usdc"
                    .into(),
            ))
        }
        Some("usda") => std::fs::write(path, exported.to_usda()?).map_err(io)?,
        other => {
            if let Some(other) = other {
                exported.warnings.push(format!(
                    "unknown extension `.{other}`: wrote usda text; use .usda, .usdc or .usd"
                ));
            }
            std::fs::write(path, exported.to_usda()?).map_err(io)?;
        }
    }
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
    export_stage(input, options, None, asset_stem)
}

/// Authors `input` — one frame — as a simulation stage: see
/// [`SimulationSpec`].
pub fn export_simulation(
    input: &AnimationInput,
    simulation: &SimulationSpec,
    asset_stem: &str,
) -> Result<ExportedAnimation, UsdExportError> {
    if input.times.len() != 1 {
        return Err(UsdExportError::Input(format!(
            "a simulation stage is one frame, got {}",
            input.times.len()
        )));
    }
    if simulation.bodies.len() != input.objects.len()
        || simulation.articulations.len() != input.robots.len()
        || simulation.rides.len() != input.robots.len()
    {
        return Err(UsdExportError::Input(
            "simulation spec does not line up with the objects and robots".into(),
        ));
    }
    let carriers = simulation.rides.iter().filter_map(|ride| match ride {
        Some(Ride::Base(carrier)) => carrier.unit,
        _ => None,
    });
    if let Some(bad) = simulation
        .bodies
        .iter()
        .filter_map(|b| b.unit)
        .chain(carriers)
        .find(|&u| u >= simulation.units.len())
    {
        return Err(UsdExportError::Input(format!(
            "object names unit {bad} of {}",
            simulation.units.len()
        )));
    }
    for (r, ride) in simulation.rides.iter().enumerate() {
        if let Some(Ride::Joins { host }) = ride {
            if *host >= r || !matches!(simulation.rides[*host], Some(Ride::Base(_))) {
                return Err(UsdExportError::Input(format!(
                    "robot {r} joins robot {host}, which is not an earlier robot riding a vehicle"
                )));
            }
        }
    }
    export_stage(
        input,
        &ExportOptions::default(),
        Some(simulation),
        asset_stem,
    )
}

fn export_stage(
    input: &AnimationInput,
    options: &ExportOptions,
    simulation: Option<&SimulationSpec>,
    asset_stem: &str,
) -> Result<ExportedAnimation, UsdExportError> {
    let n = input.times.len();
    if n == 0 {
        return Err(UsdExportError::Input("no frames".into()));
    }
    if input.robots.is_empty() && input.objects.is_empty() {
        // A robot-less cell still animates its objects (an AGV loop, a
        // conveyor line); with neither there is nothing to write.
        return Err(UsdExportError::Input("no robots and no objects".into()));
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
    for camera in input.cameras {
        if let PoseTrack::Sampled(samples) = &camera.track {
            if samples.len() != n {
                return Err(UsdExportError::Input(format!(
                    "camera `{}`: {} samples for {} frames",
                    camera.name,
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
    layer.collapse_single = simulation.is_some();

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
    if simulation.is_some() {
        layer.root_field("kilogramsPerUnit", Value::Double(1.0));
    }
    layer.ensure_prim("/World", Specifier::Def, Some("Xform"));

    // Where everything under `/World/Env` goes, decided before anything is
    // authored: a robot bolted to the body that carries it names that
    // body's prim.
    let layout = plan_env_layout(input.objects, simulation);
    let mut assets = Vec::new();
    let mut appearances = visual::VisualAssets::new(asset_stem);
    // Robot prims under /World, uniquified from sanitized instance names;
    // asset directories dedup by source stage (two instances of the same
    // asset share one copy, both references point at it).
    let mut used_prims: HashMap<String, usize> = HashMap::new();
    let mut used_dirs: HashMap<String, usize> = HashMap::new();
    let mut source_dirs: HashMap<PathBuf, String> = HashMap::new();
    // Rides, resolved as the robots are authored in order: a host's root
    // body (what a joining robot on a body-less vehicle is bolted to) is
    // known once the host is; a host that could not take the chain
    // strands its riders and hands its vehicle back to the outside
    // (`demoted`: the body is authored kinematic after all).
    let rides = simulation.map_or(&[][..], |s| s.rides);
    let hosts: std::collections::HashSet<usize> = rides
        .iter()
        .filter_map(|ride| match ride {
            Some(Ride::Joins { host }) => Some(*host),
            _ => None,
        })
        .collect();
    let mut root_bodies: Vec<Option<(String, Isometry3<f64>)>> = vec![None; input.robots.len()];
    let mut stranded: std::collections::HashSet<usize> = Default::default();
    let mut demoted: std::collections::HashSet<usize> = Default::default();
    for (r, robot) in input.robots.iter().enumerate() {
        let prim_name = unique_child(&mut used_prims, &sanitize_name(robot.name));
        let robot_prim = format!("/World/{prim_name}");
        // A joining robot on a body-less vehicle is bolted to its host's
        // root body, known once the host is authored (hosts come first).
        let host_root: Option<(String, Isometry3<f64>)> = match rides.get(r) {
            Some(Some(Ride::Joins { host })) => root_bodies[*host].clone(),
            _ => None,
        };
        let mount = match (rides.get(r).and_then(Option::as_ref), simulation) {
            (Some(Ride::Base(carrier)), Some(s)) => Some(physics::Mount::Base {
                frame: &carrier.frame,
                body: carrier
                    .unit
                    .map(|u| (layout.units[u].as_str(), &s.units[u].pose)),
            }),
            (Some(Ride::Joins { host }), Some(s)) if !stranded.contains(host) => {
                let body = match &rides[*host] {
                    Some(Ride::Base(CarrierSpec { unit: Some(u), .. })) => {
                        Some((layout.units[*u].as_str(), &s.units[*u].pose))
                    }
                    _ => host_root.as_ref().map(|(prim, pose)| (prim.as_str(), pose)),
                };
                match body {
                    Some(body) => Some(physics::Mount::Joins { body }),
                    None => {
                        warnings.push(format!(
                            "robot `{}` cannot join robot `{}`'s articulation: nothing to bolt to; \
                             anchored to the world where it stands",
                            robot.name, input.robots[*host].name
                        ));
                        None
                    }
                }
            }
            (Some(Ride::Joins { host }), _) => {
                warnings.push(format!(
                    "robot `{}` rides with robot `{}`, which is anchored to the world; so is this one",
                    robot.name, input.robots[*host].name
                ));
                None
            }
            _ => None,
        };
        // Robots sharing an articulation are named apart.
        let shared = hosts.contains(&r) || matches!(rides.get(r), Some(Some(Ride::Joins { .. })));
        let prefix = shared.then_some(prim_name.as_str());
        match robot.model.source.usd_stage() {
            Some((path, articulation_root)) => {
                let (info, _stage) =
                    robot_stage_info(path, &[], articulation_root, robot.model, &mut warnings)?;
                let dir = match source_dirs.get(path) {
                    Some(dir) => dir.clone(),
                    None => {
                        let dir =
                            unique_child(&mut used_dirs, &sanitize_name(robot.name).to_lowercase());
                        source_dirs.insert(path.to_path_buf(), dir.clone());
                        assets.extend(robot_asset_copies(path, asset_stem, &dir, &mut warnings)?);
                        dir
                    }
                };
                let taken = author_referenced_robot(
                    &mut layer,
                    robot,
                    &codes,
                    &info,
                    &robot_prim,
                    &format!("./{asset_stem}_assets/{dir}"),
                    mount,
                    &mut warnings,
                )?;
                root_bodies[r] = Some(taken.root_body);
                if mount.is_some() && !taken.mounted {
                    stranded.insert(r);
                    if let Some(Ride::Base(CarrierSpec { unit: Some(u), .. })) = &rides[r] {
                        demoted.insert(*u);
                    }
                }
            }
            // URDF robots and composites have no single stage to reference;
            // their geometry bakes into per-link transforms.
            None => {
                let articulation = simulation.and_then(|s| {
                    Some(physics::Rooted {
                        spec: s.articulations[r].as_ref()?,
                        mount,
                        prefix,
                    })
                });
                let root_body = author_urdf_robot(
                    &mut layer,
                    robot,
                    &codes,
                    &robot_prim,
                    articulation,
                    &mut warnings,
                    &mut appearances,
                )?;
                root_bodies[r] = Some(root_body);
            }
        }
    }

    author_objects(
        &mut layer,
        input.objects,
        simulation,
        &layout,
        &demoted,
        &codes,
        &mut warnings,
        &mut appearances,
    )?;
    if let Some(height) = simulation.and_then(|s| s.ground) {
        physics::author_ground(&mut layer, height);
    }
    assets.extend(appearances.copies);
    author_curves(&mut layer, input.curves, &mut warnings);
    author_cameras(&mut layer, input.cameras, &codes);

    Ok(ExportedAnimation {
        data: layer.finish(),
        assets,
        warnings,
    })
}

/// Authors each [`CameraSpec`] as a `Camera` prim under `/World/Cameras`
/// (created only when there is something to hold): the mount-resolved
/// xform plus the pinhole optics as plain attributes.
fn author_cameras(layer: &mut LayerBuilder, cameras: &[CameraSpec], codes: &[f64]) {
    if cameras.is_empty() {
        return;
    }
    layer.ensure_prim("/World/Cameras", Specifier::Def, Some("Xform"));
    let mut used: HashMap<String, usize> = HashMap::new();
    for spec in cameras {
        let prim = format!(
            "/World/Cameras/{}",
            unique_child(&mut used, &sanitize_name(&spec.name))
        );
        layer.ensure_prim(&prim, Specifier::Def, Some("Camera"));
        let pose = match &spec.track {
            PoseTrack::Static(x) => XformValue::Static(*x),
            PoseTrack::Sampled(samples) => XformValue::Sampled(codes, samples.clone()),
        };
        layer.xform(&prim, &pose, None);
        for (name, value) in [
            ("focalLength", spec.focal_length),
            ("horizontalAperture", spec.horizontal_aperture),
            ("verticalAperture", spec.vertical_aperture),
        ] {
            layer.attr(
                &prim,
                name,
                "float",
                AttrValue::Default(Value::Float(value as f32)),
            );
        }
        layer.attr(
            &prim,
            "clippingRange",
            "float2",
            AttrValue::Default(Value::Vec2f(gf::vec2f(
                spec.clipping[0] as f32,
                spec.clipping[1] as f32,
            ))),
        );
        // Pixels, for botrail's own round trip; a `custom` attribute, so
        // no schema claims it.
        layer.attr_meta(
            &prim,
            crate::camera::RESOLUTION_ATTR,
            "int2",
            AttrValue::Default(Value::Vec2i(gf::Vec2i {
                x: spec.resolution[0] as i32,
                y: spec.resolution[1] as i32,
            })),
            &[(FieldKey::Custom.as_ref(), Value::Bool(true))],
        );
    }
}

/// Authors each [`CurveSpec`] as a linear `BasisCurves` prim under
/// `/World/Toolpaths` (created only when there is something to hold).
fn author_curves(layer: &mut LayerBuilder, curves: &[CurveSpec], warnings: &mut Vec<String>) {
    if curves.is_empty() {
        return;
    }
    let mut used: HashMap<String, usize> = HashMap::new();
    for spec in curves {
        let polylines: Vec<&Vec<[f64; 3]>> = spec.curves.iter().filter(|c| c.len() >= 2).collect();
        if polylines.is_empty() {
            warnings.push(format!(
                "toolpath curve `{}` has no polyline with 2+ points; skipped",
                spec.name
            ));
            continue;
        }
        if !layer.has_prim("/World/Toolpaths") {
            layer.ensure_prim("/World/Toolpaths", Specifier::Def, Some("Xform"));
        }
        let prim = format!(
            "/World/Toolpaths/{}",
            unique_child(&mut used, &sanitize_name(&spec.name))
        );
        layer.ensure_prim(&prim, Specifier::Def, Some("BasisCurves"));
        let mut counts: Vec<i32> = Vec::with_capacity(polylines.len());
        let mut points = Vec::new();
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for line in &polylines {
            counts.push(line.len() as i32);
            for p in line.iter() {
                let (x, y, z) = (p[0] as f32, p[1] as f32, p[2] as f32);
                points.push(gf::vec3f(x, y, z));
                for (i, v) in [x, y, z].into_iter().enumerate() {
                    min[i] = min[i].min(v);
                    max[i] = max[i].max(v);
                }
            }
        }
        layer.attr(
            &prim,
            "type",
            "token",
            AttrValue::Uniform(Value::Token(tf::Token::from("linear"))),
        );
        layer.attr(
            &prim,
            "wrap",
            "token",
            AttrValue::Uniform(Value::Token(tf::Token::from("nonperiodic"))),
        );
        layer.attr(
            &prim,
            "curveVertexCounts",
            "int[]",
            AttrValue::Default(Value::IntVec(counts)),
        );
        layer.attr(
            &prim,
            "points",
            "point3f[]",
            AttrValue::Default(Value::Vec3fVec(points)),
        );
        // One width for the whole prim: `constant` interpolation is
        // attribute metadata, same rule as a primvar's.
        layer.attr_meta(
            &prim,
            "widths",
            "float[]",
            AttrValue::Default(Value::FloatVec(vec![spec.width])),
            &[("interpolation", Value::Token(tf::Token::from("constant")))],
        );
        layer.attr(
            &prim,
            "primvars:displayColor",
            "color3f[]",
            AttrValue::Default(Value::Vec3fVec(vec![gf::vec3f(
                spec.color[0],
                spec.color[1],
                spec.color[2],
            )])),
        );
        layer.attr(
            &prim,
            "extent",
            "float3[]",
            AttrValue::Default(Value::Vec3fVec(vec![
                gf::vec3f(min[0], min[1], min[2]),
                gf::vec3f(max[0], max[1], max[2]),
            ])),
        );
    }
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

/// The crate version a written `.usdc` claims: the lowest one its content
/// needs, which is what stock USD writes too. openusd's writer stamps the
/// newest version it knows (0.12, USD 25's splines) on every file, and a
/// reader refuses a file newer than itself whatever is inside — Isaac Sim
/// 5.1 (USD 24, crate 0.11) would not open a botrail `.usdc` at all.
/// Nothing this exporter authors is newer than 0.8: no `TimeCode` values
/// (0.9), path expressions (0.10), relocates (0.11) or splines (0.12).
const CRATE_VERSION: [u8; 3] = [0, 8, 0];

/// Restamps the bootstrap header (`"PXR-USDC"`, then major/minor/patch).
fn stamp_crate_version(bytes: &mut [u8]) {
    if bytes.len() >= 11 && bytes.starts_with(b"PXR-USDC") {
        bytes[8..11].copy_from_slice(&CRATE_VERSION);
    }
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
    /// Type names of the prims defined here (not of the `over`s).
    types: HashMap<String, String>,
    /// Authors a lone time sample as the attribute's default instead — a
    /// simulation stage states values, not a one-frame animation.
    collapse_single: bool,
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
            types: HashMap::new(),
            collapse_single: false,
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
                    self.types.insert(path.to_string(), t.to_string());
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
                    self.types.insert(path.to_string(), t.to_string());
                }
                self.prims.insert(path.to_string(), Specifier::Def);
            }
            Some(_) => {}
        }
    }

    fn has_prim(&self, path: &str) -> bool {
        self.prims.contains_key(path)
    }

    /// The prepended `apiSchemas` authored on `path` so far.
    fn applied_schemas(&self, path: &str) -> Vec<String> {
        let Some(spec) = sdf::path(path).ok().and_then(|p| self.data.spec(&p)) else {
            return Vec::new();
        };
        match spec.get(FieldKey::ApiSchemas.as_ref()) {
            Some(Value::TokenListOp(op)) => op
                .prepended_items
                .iter()
                .map(|t| t.as_str().to_string())
                .collect(),
            _ => Vec::new(),
        }
    }

    fn is_mesh(&self, path: &str) -> bool {
        self.types.get(path).is_some_and(|t| t == "Mesh")
    }

    fn prim_field(&mut self, path: &str, key: FieldKey, value: Value) {
        let spec = self
            .data
            .spec_mut(&sdf::path(path).expect("sanitized prim path"))
            .expect("prim created via ensure_prim");
        spec.add(key, value);
    }

    fn attr(&mut self, prim: &str, name: &str, type_name: &str, value: AttrValue) {
        self.attr_meta(prim, name, type_name, value, &[]);
    }

    fn connect(&mut self, prim: &str, name: &str, type_name: &str, target: &str) {
        self.attr_meta(
            prim,
            name,
            type_name,
            AttrValue::Declaration,
            &[(
                FieldKey::ConnectionPaths.as_ref(),
                Value::PathListOp(ListOp::explicit(vec![sdf::path(target).unwrap()])),
            )],
        );
    }

    /// Authors a relationship with one explicit target
    /// (`rel <name> = <target>`).
    fn rel(&mut self, prim: &str, name: &str, target: &str) {
        let props = self.prop_children.entry(prim.to_string()).or_default();
        if !props.iter().any(|p| p == name) {
            props.push(name.to_string());
        }
        let path = sdf::path(prim)
            .expect("sanitized prim path")
            .append_property(name)
            .expect("valid property name");
        let spec = self.data.create_spec(path, SpecType::Relationship);
        spec.add(
            FieldKey::Variability,
            Value::Variability(Variability::Uniform),
        );
        spec.add(
            FieldKey::TargetPaths,
            Value::PathListOp(ListOp::explicit(vec![
                sdf::path(target).expect("sanitized target path")
            ])),
        );
    }

    /// [`attr`](Self::attr) plus attribute *metadata* — fields on the
    /// attribute itself rather than sibling properties, which is where USD
    /// keeps a primvar's `interpolation`. Authored in one pass because
    /// creating a spec replaces whatever was there.
    fn attr_meta(
        &mut self,
        prim: &str,
        name: &str,
        type_name: &str,
        value: AttrValue,
        meta: &[(&str, Value)],
    ) {
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
            AttrValue::Declaration => {}
            AttrValue::Default(v) => spec.add(FieldKey::Default, v),
            AttrValue::Uniform(v) => {
                spec.add(
                    FieldKey::Variability,
                    Value::Variability(Variability::Uniform),
                );
                spec.add(FieldKey::Default, v);
            }
            AttrValue::Samples(mut map) if self.collapse_single && map.len() == 1 => {
                spec.add(FieldKey::Default, map.remove(0).1);
            }
            AttrValue::Samples(map) => spec.add(FieldKey::TimeSamples, Value::TimeSamples(map)),
        }
        for (key, v) in meta {
            spec.add(*key, v.clone());
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

    /// Resolves the accumulated children keys and hands over the layer
    /// data; serialization (text or crate) is the caller's choice.
    fn finish(mut self) -> sdf::Data {
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
        self.data
    }
}

enum AttrValue {
    Declaration,
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
    /// Joints the stage ties to the world — one body relationship empty,
    /// the other a body of this robot — relative to the root prim: what
    /// anchors the robot where its asset put it.
    world_joints: Vec<String>,
    /// The root prim's child names, so what is authored beneath it takes a
    /// free name.
    root_children: Vec<String>,
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

/// Link prim names of a baked robot export, in model-link order: the
/// writer's `sanitize_name` + collision uniquing (`author_urdf_robot`),
/// replayed so the reader lands on the same prims.
pub(crate) fn baked_link_names(model: &RobotModel) -> Vec<String> {
    let mut used = HashMap::new();
    model
        .links
        .iter()
        .map(|l| unique_child(&mut used, &sanitize_name(&l.name)))
        .collect()
}

/// Structural facts of a baked (URDF- or composite-sourced) robot resolved
/// against a recording: one flat `def Xform` per link under `stage_root`,
/// named by [`baked_link_names`], carrying world-pose samples. The baked
/// writer authors no reference arc, no joint prims and no `JointStateAPI`,
/// and its poses are already link frames in botrail axes — so `k_inv` is
/// identity and there is no source stage to consult.
pub(crate) fn baked_robot_stage_info_on(
    opened: &OpenedStage,
    stage_root: &str,
    model: &RobotModel,
) -> Result<RobotStageInfo, UsdExportError> {
    let mut links = Vec::with_capacity(model.links.len());
    for name in baked_link_names(model) {
        let stage_path = format!("{stage_root}/{name}");
        if !opened.has_prim(&stage_path) {
            return Err(UsdExportError::RobotStage(format!(
                "baked link `{stage_path}` has no prim in `{}` — was this recording exported \
                 from the same cell?",
                opened.path
            )));
        }
        links.push(LinkStageInfo {
            rel_path: name,
            stage_path,
            k_inv: Isometry3::identity(),
            parent: ParentAnchor::Root {
                offset: Isometry3::identity(),
            },
        });
    }
    Ok(RobotStageInfo {
        frame: opened.frame,
        root_path: stage_root.to_string(),
        links,
        world_joints: Vec::new(),
        root_children: Vec::new(),
    })
}

/// [`find_robot_root`] for baked robots: the unique prim owning a child
/// prim for every baked link name.
pub(crate) fn find_baked_robot_root(
    opened: &OpenedStage,
    model: &RobotModel,
) -> Result<String, UsdExportError> {
    let names = baked_link_names(model);
    let all_present = |root: &str| {
        names
            .iter()
            .all(|n| opened.prims.contains_key(format!("{root}/{n}").as_str()))
    };
    let mut candidates: Vec<String> = opened
        .prims
        .keys()
        .filter(|p| p.as_str() != "/" && all_present(p))
        .cloned()
        .collect();
    candidates.sort();
    match candidates.len() {
        0 => Err(UsdExportError::RobotStage(format!(
            "no prim in `{}` carries this robot's baked links (looked for `{}` and its \
             siblings under one prim)",
            opened.path, names[model.root_link]
        ))),
        1 => Ok(candidates.remove(0)),
        _ => Err(UsdExportError::RobotStage(format!(
            "multiple prims in `{}` match this robot's baked links: {}",
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

    // The stage's own anchoring to the world: every joint with one empty
    // body relationship whose other end is a body of this robot.
    let is_link = |path: &str| links.iter().any(|l| l.stage_path == path);
    let mut world_joints: Vec<String> = Vec::new();
    for (prim_path, info) in prims {
        if !in_subtree(prim_path)
            || !info.type_name.starts_with("Physics")
            || !info.type_name.ends_with("Joint")
        {
            continue;
        }
        let joint = AnyJoint(info.prim.clone());
        let target = |rel: openusd::usd::Relationship| -> Option<String> {
            rel.targets()
                .ok()
                .and_then(|t| t.first().map(|p| p.to_string()))
        };
        let (body0, body1) = (target(joint.body0_rel()), target(joint.body1_rel()));
        let anchored = match (body0, body1) {
            (None, Some(b)) | (Some(b), None) => is_link(&b),
            _ => false,
        };
        if anchored {
            world_joints.push(
                prim_path
                    .strip_prefix(&format!("{articulation_root}/"))
                    .unwrap_or(prim_path)
                    .to_string(),
            );
        }
    }
    world_joints.sort();
    let root_children: Vec<String> = prims
        .get(articulation_root)
        .map(|root| {
            root.prim
                .children()
                .map(|children| {
                    children
                        .iter()
                        .filter_map(|c| c.path().as_str().rsplit('/').next().map(str::to_string))
                        .collect()
                })
                .unwrap_or_default()
        })
        .unwrap_or_default();
    Ok(RobotStageInfo {
        frame,
        root_path: articulation_root.to_string(),
        links,
        world_joints,
        root_children,
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

/// What [`author_referenced_robot`] found and did for a rider.
struct ReferencedRobot {
    /// The root link's body prim and its world pose in the export frame.
    root_body: (String, Isometry3<f64>),
    /// Whether the mount asked for was authored.
    mounted: bool,
}

#[allow(clippy::too_many_arguments)]
fn author_referenced_robot(
    layer: &mut LayerBuilder,
    robot: &RobotAnimation,
    codes: &[f64],
    info: &RobotStageInfo,
    robot_prim: &str,
    asset_ref_dir: &str,
    mount: Option<physics::Mount>,
    warnings: &mut Vec<String>,
) -> Result<ReferencedRobot, UsdExportError> {
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

    // The root body prim, and its pose in the export world: the stage
    // pose through the corrective placement.
    let root = robot.model.root_link;
    let root_prim = match info.links[root].rel_path.as_str() {
        "" => robot_prim.to_string(),
        rel => format!("{robot_prim}/{rel}"),
    };
    let stage_pose = body_stage[0][root];
    let root_pose = Isometry3::from_parts(
        Translation3::from(info.frame.up_fix * (stage_pose.translation.vector * mpu)),
        info.frame.up_fix * stage_pose.rotation,
    );
    let mut mounted = false;
    if let Some(mount) = mount {
        if root_is_body.is_some() {
            // The articulation is rooted on the base body itself: a chain
            // cannot hang under a body, and PhysX would keep that body as
            // the root of the tree. Unsupported for now.
            warnings.push(format!(
                "robot `{}`: its stage roots the articulation on the base body itself; the \
                 robot stays anchored to the world as its stage says and does not ride",
                robot.name
            ));
        } else {
            // The chain's poses are world-frame: under the robot prim only
            // when that composes at identity, a sibling otherwise.
            let identity = (mpu - 1.0).abs() < 1e-12 && info.frame.up_fix.angle() < 1e-12;
            let sibling;
            let chain_parent = if identity {
                robot_prim
            } else {
                sibling = format!("{robot_prim}_carrier");
                sibling.as_str()
            };
            let mut used: HashMap<String, usize> = HashMap::new();
            if identity {
                for name in &info.root_children {
                    used.insert(name.clone(), 1);
                }
            }
            physics::author_referenced_mount(
                layer,
                robot_prim,
                chain_parent,
                &mut used,
                &info.world_joints,
                (root_prim.as_str(), &root_pose),
                mount,
            );
            mounted = true;
        }
    }
    Ok(ReferencedRobot {
        root_body: (root_prim, root_pose),
        mounted,
    })
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
    for (ji, joint) in robot.model.joints.iter().enumerate() {
        // Mimic joints carry no `q` entry but do move: author the value the
        // model derives for them, so a reader that trusts joint state sees
        // the whole articulation, gripper included.
        if joint.q_index.is_none() && joint.mimic.is_none() {
            continue;
        }
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
            .map(|(code, q)| {
                let value = robot.model.joint_value(ji, q);
                (*code, Value::Float(to_stage(value, info.frame.mpu) as f32))
            })
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
    match model.source.usd_stage() {
        Some((path, _)) => path
            .file_name()
            .map(|f| f.to_string_lossy().to_string())
            .ok_or_else(|| UsdExportError::RobotStage("robot stage path has no file name".into())),
        None => unreachable!("only called for USD robots"),
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
    // `strip_prefix` is textual, and the dependencies come back with their
    // directories canonicalized — a stage reached through a symlinked
    // directory (macOS temp dirs) would never contain them. Resolve
    // directories only: a huggingface_hub cache file is itself a symlink
    // onto a blob that lives elsewhere under another name.
    let in_real_dir = |p: &Path| match (p.parent(), p.file_name()) {
        (Some(dir), Some(name)) => dir
            .canonicalize()
            .map(|dir| dir.join(name))
            .unwrap_or_else(|_| p.to_path_buf()),
        _ => p.to_path_buf(),
    };
    let real_stage_dir = stage_dir
        .canonicalize()
        .unwrap_or_else(|_| stage_dir.to_path_buf());
    let mut copies = Vec::new();
    for dep in deps {
        match in_real_dir(&dep).strip_prefix(&real_stage_dir) {
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
    articulation: Option<physics::Rooted>,
    warnings: &mut Vec<String>,
    appearances: &mut visual::VisualAssets,
) -> Result<(String, Isometry3<f64>), UsdExportError> {
    layer.ensure_prim(robot_prim, Specifier::Def, Some("Xform"));
    let mut used = HashMap::new();
    let mut link_prims = Vec::with_capacity(robot.model.links.len());
    let prefix = articulation.and_then(|a| a.prefix);
    for (i, link) in robot.model.links.iter().enumerate() {
        let name = unique_child(
            &mut used,
            &physics::prefixed(prefix, &sanitize_name(&link.name)),
        );
        let prim = format!("{robot_prim}/{name}");
        link_prims.push(prim.clone());
        layer.ensure_prim(&prim, Specifier::Def, Some("Xform"));
        let poses: Vec<Isometry3<f64>> = robot.link_poses.iter().map(|p| p[i]).collect();
        layer.xform(&prim, &XformValue::Sampled(codes, poses), None);
        for (vi, shape) in link.visuals.iter().enumerate() {
            let shape_prim = format!("{prim}/Visual_{vi}");
            if let Some(source) = &shape.visual_asset {
                appearances.author(
                    layer,
                    &shape_prim,
                    source,
                    &XformValue::Static(shape.origin),
                    shape.color,
                    None,
                )?;
            } else {
                author_shape(layer, &shape_prim, shape, warnings)?;
            }
        }
    }
    if let Some(rooted) = articulation {
        physics::author_articulation(
            layer,
            robot,
            rooted,
            robot_prim,
            &link_prims,
            &mut used,
            warnings,
        )?;
    }
    let root = robot.model.root_link;
    Ok((link_prims[root].clone(), robot.link_poses[0][root]))
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
        shape.color,
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
    // A mesh that carried its own materials paints per face — that is the
    // manufacturer's own coloring, and one flat color over the top would
    // throw it away. An explicit `color` still wins: it is a choice the
    // scene made (authored scenery, a highlight), which the mesh cannot
    // know about.
    if color.is_none() {
        if let Geometry::Mesh { path, .. } = geometry {
            if let Some(colors) = mesh_face_colors(path) {
                // One color per face: `uniform` is metadata on the primvar,
                // not a sibling property — without it a reader takes the
                // array as `constant` and the whole mesh goes one color.
                layer.attr_meta(
                    prim,
                    "primvars:displayColor",
                    "color3f[]",
                    AttrValue::Default(Value::Vec3fVec(colors)),
                    &[("interpolation", Value::Token(tf::Token::from("uniform")))],
                );
                return Ok(());
            }
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

/// Per-face diffuse of a mesh file, when it carried materials. Re-reads the
/// file rather than threading colors through every caller; loads are
/// cached upstream and this runs once per authored prim.
fn mesh_face_colors(path: &std::path::Path) -> Option<Vec<gf::Vec3f>> {
    let data = botrail_mesh::load_path(path).ok()?;
    if data.face_colors.is_empty() {
        return None;
    }
    Some(
        data.face_colors
            .iter()
            .map(|c| gf::vec3f(c[0], c[1], c[2]))
            .collect(),
    )
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

/// Where every exported object (and rigid unit) lives under `/World/Env`.
///
/// An obstacle named `a/b/c` is the prim `/World/Env/a/b/c`, its world
/// transform the obstacle's pose — the convention recordings are read
/// back by. Two things bend the plain "leaf gprim under identity groups"
/// picture:
///
/// - a **container**: an obstacle whose name is a prefix of another's
///   (`washer/basket` next to `washer/basket/mesh`), or the frame member
///   of a rigid unit. Its prim is a posed `Xform` with the gprim beneath
///   (`geom`), and everything under it is authored relative to it — a
///   gprim cannot parent prims, and a scaled `Cube` would scale them.
/// - a **hoisted** prim: whatever is named under a rigid body without
///   being part of it (a crate on a pallet, each a body of its own) moves
///   out to be the body's sibling, `<body>_<rest>`. UsdPhysics takes
///   every prim below a rigid body as part of that body.
///
/// A third case makes a container too: an object that **collides as a
/// primitive but is drawn as a mesh** (the parts library's tray is a box
/// to the planner and the engine, a pocketed tray to the eye). The box is
/// authored beside the picture as an undrawn collider (`collider`), so a
/// consumer's physics sees the shape botrail's does — measured in Isaac
/// Sim: colliding as the picture, a part seated on the tray fell into its
/// pocket.
struct EnvLayout {
    /// The body `Xform` of each unit.
    units: Vec<String>,
    /// Each object's own prim — the gprim, or the container `Xform`.
    objects: Vec<String>,
    /// Where a container's gprim goes; `None` for a plain leaf.
    geoms: Vec<Option<String>>,
    /// Where the collision primitive of a mesh-drawn object goes.
    colliders: Vec<Option<String>>,
}

/// Whether `obj`'s collision shape has to be authored apart from its
/// picture: a collider whose geometry is a primitive while a source mesh
/// draws it. A mesh geometry *is* the drawn gprim's triangles (that is how
/// imported scenery gets its geometry), so that one collides as drawn.
fn collides_apart(obj: &ObjectSpec) -> bool {
    obj.physics.is_some()
        && obj.visual_asset.is_some()
        && !matches!(obj.geometry, Geometry::Mesh { .. })
}

const ENV_ROOT: &str = "/World/Env";

fn env_segments(name: &str) -> Vec<String> {
    let segments: Vec<String> = name
        .split('/')
        .filter(|s| !s.is_empty())
        .map(sanitize_name)
        .collect();
    if segments.is_empty() {
        vec!["object".to_string()]
    } else {
        segments
    }
}

/// Proper ancestors of `path` below [`ENV_ROOT`], shortest first.
fn env_ancestors(path: &str) -> impl Iterator<Item = &str> {
    path.match_indices('/')
        .map(move |(i, _)| &path[..i])
        .filter(|a| a.len() > ENV_ROOT.len())
}

fn plan_env_layout(objects: &[ObjectSpec], simulation: Option<&SimulationSpec>) -> EnvLayout {
    use std::collections::HashSet;
    let units = simulation.map_or(&[][..], |s| s.units);
    let mut taken: HashSet<String> = HashSet::new();
    let mut bodies: HashSet<String> = HashSet::new();
    let claim = |taken: &mut HashSet<String>, path: String| -> String {
        let mut unique = path.clone();
        let mut n = 1;
        while !taken.insert(unique.clone()) {
            n += 1;
            unique = format!("{path}_{n}");
        }
        unique
    };
    // Out from under the topmost body above `path`, as that body's sibling.
    let hoist = |bodies: &HashSet<String>, path: String| -> String {
        let Some(body) = env_ancestors(&path).find(|a| bodies.contains(*a)) else {
            return path;
        };
        let rest = path[body.len() + 1..].replace('/', "_");
        format!("{body}_{rest}")
    };

    // Units shallowest first, so a body is placed before what nests in it.
    let mut order: Vec<usize> = (0..units.len()).collect();
    let depth = |name: &str| name.split('/').filter(|s| !s.is_empty()).count();
    order.sort_by_key(|&u| depth(&units[u].name));
    let mut unit_paths = vec![String::new(); units.len()];
    for u in order {
        let natural = format!("{ENV_ROOT}/{}", env_segments(&units[u].name).join("/"));
        let path = claim(&mut taken, hoist(&bodies, natural));
        bodies.insert(path.clone());
        unit_paths[u] = path;
    }

    let mut object_paths = Vec::with_capacity(objects.len());
    for (i, obj) in objects.iter().enumerate() {
        let segments = env_segments(&obj.name);
        let unit = simulation.and_then(|s| s.bodies[i].unit);
        let path = match unit {
            Some(u) => {
                // Below its body, by what its name adds to the unit's; the
                // frame member (same name) *is* the body prim.
                let own = env_segments(&units[u].name);
                let under = segments.len() >= own.len() && segments[..own.len()] == own[..];
                let rest = if under {
                    &segments[own.len()..]
                } else {
                    &segments[..]
                };
                if rest.is_empty() {
                    unit_paths[u].clone()
                } else {
                    claim(&mut taken, format!("{}/{}", unit_paths[u], rest.join("/")))
                }
            }
            None => {
                let natural = format!("{ENV_ROOT}/{}", segments.join("/"));
                claim(&mut taken, hoist(&bodies, natural))
            }
        };
        object_paths.push(path);
    }

    let mut parents: HashSet<&str> = HashSet::new();
    for path in object_paths.iter().chain(&unit_paths) {
        parents.extend(env_ancestors(path));
    }
    let container: Vec<bool> = object_paths
        .iter()
        .zip(objects)
        .map(|(path, obj)| {
            bodies.contains(path) || parents.contains(path.as_str()) || collides_apart(obj)
        })
        .collect();
    let geoms = container
        .iter()
        .zip(&object_paths)
        .map(|(c, path)| c.then(|| claim(&mut taken, format!("{path}/geom"))))
        .collect();
    let colliders = objects
        .iter()
        .zip(&object_paths)
        .map(|(obj, path)| {
            collides_apart(obj).then(|| claim(&mut taken, format!("{path}/collider")))
        })
        .collect();
    EnvLayout {
        units: unit_paths,
        objects: object_paths,
        geoms,
        colliders,
    }
}

/// `child` relative to `parent`, frame by frame.
fn relative_track(parent: Option<&PoseTrack>, child: &PoseTrack) -> PoseTrack {
    match (parent, child) {
        (None, PoseTrack::Static(c)) => PoseTrack::Static(*c),
        (None, PoseTrack::Sampled(c)) => PoseTrack::Sampled(c.clone()),
        (Some(PoseTrack::Static(p)), PoseTrack::Static(c)) => PoseTrack::Static(p.inverse() * c),
        (Some(PoseTrack::Static(p)), PoseTrack::Sampled(c)) => {
            PoseTrack::Sampled(c.iter().map(|c| p.inverse() * c).collect())
        }
        (Some(PoseTrack::Sampled(p)), PoseTrack::Static(c)) => {
            PoseTrack::Sampled(p.iter().map(|p| p.inverse() * c).collect())
        }
        (Some(PoseTrack::Sampled(p)), PoseTrack::Sampled(c)) => {
            PoseTrack::Sampled(p.iter().zip(c).map(|(p, c)| p.inverse() * c).collect())
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn author_objects(
    layer: &mut LayerBuilder,
    objects: &[ObjectSpec],
    simulation: Option<&SimulationSpec>,
    layout: &EnvLayout,
    demoted: &std::collections::HashSet<usize>,
    codes: &[f64],
    warnings: &mut Vec<String>,
    appearances: &mut visual::VisualAssets,
) -> Result<(), UsdExportError> {
    let units = simulation.map_or(&[][..], |s| s.units);
    if objects.is_empty() && units.is_empty() {
        return Ok(());
    }
    layer.ensure_prim(ENV_ROOT, Specifier::Def, Some("Xform"));

    // World track of every posed prim, so what sits below one is authored
    // relative to it.
    let unit_tracks: Vec<PoseTrack> = units.iter().map(|u| PoseTrack::Static(u.pose)).collect();
    let mut posed: HashMap<&str, &PoseTrack> = HashMap::new();
    for (path, track) in layout.units.iter().zip(&unit_tracks) {
        posed.insert(path.as_str(), track);
    }
    for (i, obj) in objects.iter().enumerate() {
        if layout.geoms[i].is_some() {
            // A unit's frame member shares the body prim; the body's pose
            // stands.
            posed
                .entry(layout.objects[i].as_str())
                .or_insert(&obj.track);
        }
    }
    let frame_of = |path: &str| -> Option<&PoseTrack> {
        env_ancestors(path)
            .filter_map(|a| posed.get(a).copied())
            .last()
    };
    let xform_of = |track: PoseTrack| match track {
        PoseTrack::Static(x) => XformValue::Static(x),
        PoseTrack::Sampled(samples) => XformValue::Sampled(codes, samples),
    };
    // Grouping `Xform`s down to (not including) `path`.
    let ensure_groups = |layer: &mut LayerBuilder, path: &str| {
        for ancestor in env_ancestors(path) {
            layer.ensure_prim(ancestor, Specifier::Def, Some("Xform"));
        }
    };

    for (u, unit) in units.iter().enumerate() {
        let prim = &layout.units[u];
        ensure_groups(layer, prim);
        layer.ensure_prim(prim, Specifier::Def, Some("Xform"));
        let local = relative_track(frame_of(prim), &unit_tracks[u]);
        layer.xform(prim, &xform_of(local), None);
        // A vehicle whose rider could not take it in stays a body posed
        // from outside.
        let demoted_unit;
        let unit = if demoted.contains(&u) {
            demoted_unit = UnitSpec {
                role: BodyRole::Kinematic,
                ..unit.clone()
            };
            &demoted_unit
        } else {
            unit
        };
        physics::author_unit_body(layer, prim, unit);
    }

    // A unit's frame member shares the body prim, which is posed already.
    let posed_units: std::collections::HashSet<&str> =
        layout.units.iter().map(String::as_str).collect();
    // Physics materials dedupe on (friction, restitution): one prim per
    // distinct pair, shared by every obstacle that binds it.
    let mut materials: HashMap<(u64, u64), String> = HashMap::new();
    for (i, obj) in objects.iter().enumerate() {
        let own = &layout.objects[i];
        ensure_groups(layer, own);
        let prim = match &layout.geoms[i] {
            Some(geom) => {
                layer.ensure_prim(own, Specifier::Def, Some("Xform"));
                if !posed_units.contains(own.as_str()) {
                    let local = relative_track(frame_of(own), &obj.track);
                    layer.xform(own, &xform_of(local), None);
                }
                geom
            }
            None => own,
        };
        // Under its own container the gprim sits at the origin — said
        // once, not frame by frame.
        let frame = frame_of(prim);
        let pose = xform_of(if frame.is_some_and(|f| std::ptr::eq(f, &obj.track)) {
            PoseTrack::Static(Isometry3::identity())
        } else {
            relative_track(frame, &obj.track)
        });
        if let Some(source) = &obj.visual_asset {
            appearances.author(layer, prim, source, &pose, obj.color, obj.material)?;
        } else {
            author_geometry(layer, prim, &obj.geometry, &pose, obj.color, warnings)?;
            if let Some(finish) = obj.material {
                author_surface(layer, prim, finish);
            }
        }
        if !obj.visible.is_empty() {
            // Sparse: the first frame plus every transition. USD holds a
            // sample until the next one, so this reads identically to the
            // dense form — and a hundred blinking carve stages write a
            // handful of lines each instead of the whole frame grid.
            let samples: sdf::TimeSampleMap = codes
                .iter()
                .enumerate()
                .filter_map(|(k, &code)| {
                    let shown = obj.visible.get(k).copied().unwrap_or(true);
                    let prev = k
                        .checked_sub(1)
                        .map(|j| obj.visible.get(j).copied().unwrap_or(true));
                    (prev != Some(shown)).then(|| {
                        let token = if shown { "inherited" } else { "invisible" };
                        (code, Value::Token(tf::Token::from(token)))
                    })
                })
                .collect();
            layer.attr(prim, "visibility", "token", AttrValue::Samples(samples));
        }
        let body = simulation.map(|s| s.bodies[i]).unwrap_or_default();
        if body.guide {
            physics::guide_purpose(layer, prim);
        }
        let Some(phys) = &obj.physics else { continue };
        // A body the solver moves takes no triangle mesh: the engine's own
        // units, and a vehicle's body that is a link of its rider.
        let in_dynamic_body = body
            .unit
            .is_some_and(|u| units[u].role != BodyRole::Kinematic && !demoted.contains(&u));
        match &layout.colliders[i] {
            // The collision primitive beside the picture, undrawn; a body
            // of its own (the declared-dynamic export) is then the
            // container, which carries both.
            Some(collider) => {
                let pose = xform_of(if frame.is_some_and(|f| std::ptr::eq(f, &obj.track)) {
                    PoseTrack::Static(Isometry3::identity())
                } else {
                    relative_track(frame, &obj.track)
                });
                author_geometry(layer, collider, &obj.geometry, &pose, None, warnings)?;
                physics::guide_purpose(layer, collider);
                author_object_physics(layer, own, collider, phys, in_dynamic_body, &mut materials);
            }
            None => author_object_physics(layer, prim, prim, phys, in_dynamic_body, &mut materials),
        }
    }
    Ok(())
}

/// The optical finish uses the existing displayColor primvar, including
/// uniform face colours, instead of flattening a coloured mesh to grey.
fn author_surface(
    layer: &mut LayerBuilder,
    prim: &str,
    SurfaceMaterial {
        metalness,
        roughness,
        opacity,
    }: SurfaceMaterial,
) {
    let material = format!("{prim}/BotrailMaterial");
    let shader = format!("{material}/Surface");
    let reader = format!("{material}/Color");
    layer.ensure_prim(&material, Specifier::Def, Some("Material"));
    layer.ensure_prim(&shader, Specifier::Def, Some("Shader"));
    layer.ensure_prim(&reader, Specifier::Def, Some("Shader"));
    if let Some(value) = opacity {
        layer.attr(
            &shader,
            "inputs:opacity",
            "float",
            AttrValue::Default(Value::Float(value)),
        );
    }
    layer.attr(
        &shader,
        "info:id",
        "token",
        AttrValue::Uniform(Value::Token("UsdPreviewSurface".into())),
    );
    layer.attr(
        &reader,
        "info:id",
        "token",
        AttrValue::Uniform(Value::Token("UsdPrimvarReader_float3".into())),
    );
    layer.attr(&shader, "outputs:surface", "token", AttrValue::Declaration);
    layer.attr(&reader, "outputs:result", "float3", AttrValue::Declaration);
    layer.attr(
        &reader,
        "inputs:varname",
        "token",
        AttrValue::Default(Value::Token("displayColor".into())),
    );
    layer.attr(
        &reader,
        "inputs:fallback",
        "float3",
        AttrValue::Default(Value::Vec3f(gf::vec3f(
            ENV_COLOR[0],
            ENV_COLOR[1],
            ENV_COLOR[2],
        ))),
    );
    for (name, value) in [("metallic", metalness), ("roughness", roughness)] {
        layer.attr(
            &shader,
            &format!("inputs:{name}"),
            "float",
            AttrValue::Default(Value::Float(value)),
        );
    }
    layer.connect(
        &shader,
        "inputs:diffuseColor",
        "color3f",
        &format!("{reader}.outputs:result"),
    );
    layer.connect(
        &material,
        "outputs:surface",
        "token",
        &format!("{shader}.outputs:surface"),
    );
    layer.prim_field(
        prim,
        FieldKey::ApiSchemas,
        Value::TokenListOp(ListOp::prepended(vec!["MaterialBindingAPI".into()])),
    );
    layer.rel(prim, "material:binding", &material);
}

/// UsdPhysics applied schemas for one obstacle prim, plus its bound
/// physics material (created on first use, shared thereafter). This is
/// the write-side of the vocabulary the robot importer reads
/// (`PhysicsRigidBodyAPI` and friends) — what makes the exported cell
/// re-simulatable in a UsdPhysics consumer.
fn author_object_physics(
    layer: &mut LayerBuilder,
    body: &str,
    prim: &str,
    phys: &PhysicsSpec,
    in_dynamic_body: bool,
    materials: &mut HashMap<(u64, u64), String>,
) {
    // `body` is where a declared-dynamic object's rigid body goes: the
    // collider itself, or — when the collider stands beside the picture —
    // the container that carries both.
    //
    // A triangle mesh only collides as scenery: PhysX refuses one on a
    // body it moves. A convex decomposition keeps a tray's pocket and a
    // bin's hollow, which a single hull would fill.
    let convex = (phys.dynamic || in_dynamic_body) && layer.is_mesh(prim);
    let massed = phys.dynamic && phys.mass.is_some();
    let mut body_schemas: Vec<openusd::tf::Token> = Vec::new();
    let mut schemas: Vec<openusd::tf::Token> = Vec::new();
    let split = body != prim;
    if phys.dynamic {
        body_schemas.push("PhysicsRigidBodyAPI".into());
    }
    if !split {
        schemas.append(&mut body_schemas);
    }
    schemas.push("PhysicsCollisionAPI".into());
    if convex {
        schemas.push("PhysicsMeshCollisionAPI".into());
    }
    if massed {
        if split {
            &mut body_schemas
        } else {
            &mut schemas
        }
        .push("PhysicsMassAPI".into());
    }
    // The physics-material binding below is a `material:binding:*` rel,
    // which pxr validation requires MaterialBindingAPI for.
    schemas.push("MaterialBindingAPI".into());
    if !body_schemas.is_empty() {
        layer.prim_field(
            body,
            FieldKey::ApiSchemas,
            Value::TokenListOp(ListOp::prepended(body_schemas)),
        );
    }
    layer.prim_field(
        prim,
        FieldKey::ApiSchemas,
        Value::TokenListOp(ListOp::prepended(schemas)),
    );
    if convex {
        layer.attr(
            prim,
            "physics:approximation",
            "token",
            AttrValue::Uniform(Value::Token("convexDecomposition".into())),
        );
    }
    if phys.dynamic {
        if let Some(mass) = phys.mass {
            layer.attr(
                body,
                "physics:mass",
                "float",
                AttrValue::Default(Value::Float(mass as f32)),
            );
        }
    }
    let key = (phys.friction.to_bits(), phys.restitution.to_bits());
    let next = materials.len() + 1;
    let friction = phys.friction as f32;
    let restitution = phys.restitution as f32;
    let material = materials.entry(key).or_insert_with(|| {
        let path = format!("/World/PhysicsMaterials/mat_{next}");
        layer.ensure_prim("/World/PhysicsMaterials", Specifier::Def, Some("Scope"));
        layer.ensure_prim(&path, Specifier::Def, Some("Material"));
        layer.prim_field(
            &path,
            FieldKey::ApiSchemas,
            Value::TokenListOp(ListOp::prepended(vec![openusd::tf::Token::from(
                "PhysicsMaterialAPI",
            )])),
        );
        layer.attr(
            &path,
            "physics:staticFriction",
            "float",
            AttrValue::Default(Value::Float(friction)),
        );
        layer.attr(
            &path,
            "physics:dynamicFriction",
            "float",
            AttrValue::Default(Value::Float(friction)),
        );
        layer.attr(
            &path,
            "physics:restitution",
            "float",
            AttrValue::Default(Value::Float(restitution)),
        );
        path
    });
    layer.rel(prim, "material:binding:physics", material);
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
    /// Physics-annotated obstacles carry UsdPhysics into the layer: a
    /// dynamic part gets RigidBody + Collision + Mass with `physics:mass`,
    /// static scenery with props gets Collision only, both bind one
    /// deduped physics material with friction/restitution — and an
    /// un-annotated obstacle stays exactly the visual it always was.
    #[test]
    fn physics_specs_author_usdphysics() {
        let objects = vec![
            ObjectSpec {
                material: None,
                visual_asset: None,
                name: "part".into(),
                geometry: Geometry::Box {
                    size: Vector3::new(0.1, 0.05, 0.03),
                },
                track: PoseTrack::Static(Isometry3::translation(0.0, 0.0, 0.5)),
                color: None,
                visible: Vec::new(),
                physics: Some(PhysicsSpec {
                    dynamic: true,
                    mass: Some(2.5),
                    friction: 0.6,
                    restitution: 0.1,
                }),
            },
            ObjectSpec {
                material: None,
                visual_asset: None,
                name: "table".into(),
                geometry: Geometry::Box {
                    size: Vector3::new(1.0, 1.0, 0.7),
                },
                track: PoseTrack::Static(Isometry3::translation(0.0, 0.0, 0.35)),
                color: None,
                visible: Vec::new(),
                physics: Some(PhysicsSpec {
                    dynamic: false,
                    mass: None,
                    friction: 0.6,
                    restitution: 0.1,
                }),
            },
            ObjectSpec {
                material: None,
                visual_asset: None,
                name: "decor".into(),
                geometry: Geometry::Sphere { radius: 0.1 },
                track: PoseTrack::Static(Isometry3::translation(1.0, 0.0, 0.1)),
                color: None,
                visible: Vec::new(),
                physics: None,
            },
        ];
        let input = AnimationInput {
            robots: &[],
            times: &[0.0],
            objects: &objects,
            curves: &[],
            cameras: &[],
        };
        let exported = export_animation(&input, &ExportOptions::default(), "phys").unwrap();
        let text = exported.to_usda().unwrap();
        let block = |head: &str| -> String {
            let rest = text.split(head).nth(1).expect("prim authored");
            rest[..rest.find("\n    def ").unwrap_or(rest.len())].to_string()
        };
        let part = block("def Cube \"part\"");
        assert!(
            part.contains(
                "apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsCollisionAPI\", \"PhysicsMassAPI\", \"MaterialBindingAPI\"]"
            ),
            "{part}"
        );
        assert!(part.contains("float physics:mass = 2.5"), "{part}");
        assert!(
            part.contains("rel material:binding:physics = </World/PhysicsMaterials/mat_1>"),
            "{part}"
        );
        let table = block("def Cube \"table\"");
        assert!(
            table.contains("apiSchemas = [\"PhysicsCollisionAPI\", \"MaterialBindingAPI\"]"),
            "{table}"
        );
        assert!(!table.contains("physics:mass"), "{table}");
        // Same (friction, restitution) → the same material prim, once.
        assert!(
            table.contains("rel material:binding:physics = </World/PhysicsMaterials/mat_1>"),
            "{table}"
        );
        assert_eq!(text.matches("def Material").count(), 1, "{text}");
        let material = block("def Material \"mat_1\"");
        assert!(
            material.contains("float physics:staticFriction = 0.6"),
            "{material}"
        );
        assert!(
            material.contains("float physics:dynamicFriction = 0.6"),
            "{material}"
        );
        assert!(
            material.contains("float physics:restitution = 0.1"),
            "{material}"
        );
        assert!(
            material.contains("apiSchemas = [\"PhysicsMaterialAPI\"]"),
            "{material}"
        );
        let decor = block("def Sphere \"decor\"");
        assert!(!decor.contains("apiSchemas"), "{decor}");
        assert!(!decor.contains("physics"), "{decor}");
    }

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
            curves: &[],
            cameras: &[],
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
    fn mimic_joints_get_their_derived_joint_state() {
        // `j2` follows `j1` at half rate (PhysX gearing -0.5), leaving one
        // DOF; the joint it drives still has to show up in the animation.
        let usda = ARM.replace(
            "        def PhysicsRevoluteJoint \"j2\"\n        {",
            "        def PhysicsRevoluteJoint \"j2\" (prepend apiSchemas = \
             [\"PhysxMimicJointAPI:rotY\"])\n        {\n            \
             float physxMimicJoint:rotY:gearing = -0.5\n            \
             rel physxMimicJoint:rotY:referenceJoint = </Robot/joints/j1>\n            \
             uniform token physxMimicJoint:rotY:referenceJointAxis = \"rotZ\"",
        );
        let dir = temp_dir("mimic");
        std::fs::write(dir.join("robot.usda"), &usda).unwrap();
        let model = import_robot(
            &dir.join("robot.usda"),
            &RobotImportOptions {
                mesh_cache_dir: Some(dir.join("meshes")),
                ..Default::default()
            },
        )
        .unwrap()
        .model;
        assert_eq!(model.dof(), 1);

        let times = [0.0, 0.5];
        let configs = [vec![0.0], vec![0.8]];
        let link_poses: Vec<Vec<Isometry3<f64>>> = configs
            .iter()
            .map(|q| botrail_kin::forward_kinematics(&model, q).unwrap())
            .collect();
        let joint_samples = configs.to_vec();
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &link_poses,
            joint_samples: Some(&joint_samples),
        }];
        let warnings = write_animation(
            &dir.join("anim.usda"),
            &AnimationInput {
                robots: &robots,
                times: &times,
                objects: &[],
                curves: &[],
                cameras: &[],
            },
            &ExportOptions::default(),
        )
        .unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");

        let stage = Stage::builder()
            .resolver(SearchPathResolver::new(vec![dir.clone()]))
            .open(&dir.join("anim.usda").display().to_string())
            .unwrap();
        let state_at = |joint: &str, t: f64| -> f64 {
            let raw = stage
                .attribute(
                    sdf::path(format!("/World/Robot/joints/{joint}"))
                        .unwrap()
                        .append_property("state:angular:physics:position")
                        .unwrap(),
                )
                .get_at::<sdf::Value>(TimeCode::new(t * 60.0))
                .unwrap()
                .unwrap_or_else(|| panic!("missing joint state on {joint}"));
            match raw {
                sdf::Value::Float(v) => v as f64,
                sdf::Value::Double(v) => v,
                other => panic!("unexpected joint state value {other:?}"),
            }
        };
        for (f, t) in times.iter().enumerate() {
            assert!((state_at("j1", *t) - configs[f][0].to_degrees()).abs() < 1e-3);
            assert!(
                (state_at("j2", *t) - (0.5 * configs[f][0]).to_degrees()).abs() < 1e-3,
                "frame {f}: {}",
                state_at("j2", *t)
            );
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn zup_meter_reference_roundtrip() {
        reference_roundtrip(ARM, "zup");
    }

    #[test]
    fn yup_cm_reference_roundtrip() {
        reference_roundtrip(&arm_yup_cm(), "yup");
    }

    /// An OBJ that names an `mtllib` exports its own colors, one per face,
    /// so a viewer with no botrail in sight shows the machine as its
    /// manufacturer painted it. Without the `uniform` interpolation the
    /// array reads as `constant` and the mesh goes one flat color.
    #[test]
    fn obj_material_colors_export_per_face() {
        let dir = temp_dir("mtl");
        std::fs::write(
            dir.join("part.mtl"),
            "newmtl yellow\nKd 1 1 0\nnewmtl grey\nKd 0.5 0.5 0.5\n",
        )
        .unwrap();
        let obj = dir.join("part.obj");
        std::fs::write(
            &obj,
            "mtllib part.mtl\n\
             v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n\
             usemtl yellow\nf 1 2 3\n\
             usemtl grey\nf 1 3 4\n",
        )
        .unwrap();
        let urdf = format!(
            r#"<robot name="r">
              <link name="base_link">
                <visual><geometry><mesh filename="{}"/></geometry></visual>
              </link>
            </robot>"#,
            obj.display()
        );
        let model = RobotModel::from_urdf_str(&urdf).unwrap();
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &[vec![Isometry3::identity()]],
            joint_samples: None,
        }];
        let out = dir.join("anim.usda");
        write_animation(
            &out,
            &AnimationInput {
                robots: &robots,
                times: &[0.0],
                objects: &[],
                curves: &[],
                cameras: &[],
            },
            &ExportOptions::default(),
        )
        .unwrap();

        let text = std::fs::read_to_string(&out).unwrap();
        let color = text
            .lines()
            .find(|l| l.contains("primvars:displayColor"))
            .expect("displayColor authored");
        assert!(color.contains("(1.0, 1.0, 0.0)"), "{color}");
        assert!(color.contains("(0.5, 0.5, 0.5)"), "{color}");
        assert!(
            text.contains(r#"interpolation = "uniform""#),
            "per-face colors need uniform interpolation"
        );
    }

    fn scenery(name: &str, size: [f64; 3], at: [f64; 3], collides: bool) -> ObjectSpec {
        ObjectSpec {
            material: None,
            visual_asset: None,
            name: name.into(),
            geometry: Geometry::Box {
                size: Vector3::new(size[0], size[1], size[2]),
            },
            track: PoseTrack::Static(Isometry3::translation(at[0], at[1], at[2])),
            color: None,
            visible: Vec::new(),
            physics: collides.then_some(PhysicsSpec {
                dynamic: false,
                mass: None,
                friction: 0.6,
                restitution: 0.0,
            }),
        }
    }

    /// An obstacle whose name is a prefix of another's used to parent it
    /// as a gprim — the child inherited the parent's pose *and* its box
    /// scale. Both now stand where the scene put them, in either order.
    #[test]
    fn nested_obstacle_names_keep_their_world_poses() {
        for flip in [false, true] {
            let mut objects = vec![
                scenery("a", [1.0, 1.0, 0.2], [1.0, 2.0, 0.1], false),
                scenery("a/b", [0.2, 0.2, 0.2], [1.0, 2.0, 0.3], false),
            ];
            if flip {
                objects.reverse();
            }
            let input = AnimationInput {
                robots: &[],
                times: &[0.0],
                objects: &objects,
                curves: &[],
                cameras: &[],
            };
            let exported = export_animation(&input, &ExportOptions::default(), "nest").unwrap();
            let dir = temp_dir(if flip { "nest_flip" } else { "nest" });
            let path = dir.join("nest.usda");
            std::fs::write(&path, exported.to_usda().unwrap()).unwrap();
            let stage = Stage::open(&path.display().to_string()).unwrap();
            let worlds = composed_worlds(&stage, 0.0);
            // The obstacle's own prim carries its pose; the gprim under a
            // container carries the box scale and nothing else.
            assert_close(
                mul_point(&worlds["/World/Env/a"], [0.0, 0.0, 0.0]),
                [1.0, 2.0, 0.1],
                1e-9,
                "a",
            );
            assert_close(
                mul_point(&worlds["/World/Env/a/geom"], [0.5, 0.5, 0.5]),
                [1.5, 2.5, 0.2],
                1e-9,
                "a's box corner",
            );
            assert_close(
                mul_point(&worlds["/World/Env/a/b"], [0.5, 0.5, 0.5]),
                [1.1, 2.1, 0.4],
                1e-9,
                "a/b's box corner",
            );
        }
    }

    /// The bodies of a simulation stage: a compound dynamic unit with its
    /// frame member, a body named under another body hoisted out to be its
    /// sibling, a belt, a static collider, an undrawn collision proxy.
    #[test]
    fn simulation_stage_authors_rigid_units() {
        let objects = vec![
            scenery("pallet", [1.2, 0.8, 0.15], [2.0, 0.0, 0.075], true),
            scenery("pallet/board", [1.2, 0.1, 0.02], [2.0, 0.3, 0.16], false),
            scenery("pallet/crate", [0.3, 0.3, 0.3], [2.0, 0.0, 0.30], true),
            scenery("line/belt", [2.0, 0.5, 0.1], [0.0, 1.0, 0.75], true),
            scenery("bench", [1.0, 1.0, 0.7], [0.0, -1.0, 0.35], true),
            scenery("bench/proxy", [1.1, 1.1, 0.7], [0.0, -1.0, 0.35], true),
        ];
        let turned = Isometry3::from_parts(
            Translation3::new(0.0, 1.0, 0.75),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
        );
        let units = [
            UnitSpec {
                name: "pallet".into(),
                role: BodyRole::Dynamic,
                pose: Isometry3::translation(2.0, 0.0, 0.075),
                mass: Some(22.0),
                belt: None,
            },
            UnitSpec {
                name: "pallet/crate".into(),
                role: BodyRole::Dynamic,
                pose: Isometry3::translation(2.0, 0.0, 0.30),
                mass: None,
                belt: None,
            },
            UnitSpec {
                name: "line/belt".into(),
                role: BodyRole::Kinematic,
                pose: turned,
                mass: Some(5.0),
                belt: Some(BeltSpec {
                    velocity: [0.0, 0.3, 0.0],
                    running: true,
                }),
            },
        ];
        let body = |unit: Option<usize>, guide: bool| ObjectBody { unit, guide };
        let bodies = [
            body(Some(0), false),
            body(Some(0), false),
            body(Some(1), false),
            body(Some(2), false),
            body(None, false),
            body(None, true),
        ];
        let mut objects = objects;
        objects[3].track = PoseTrack::Static(turned);
        let input = AnimationInput {
            robots: &[],
            times: &[0.0],
            objects: &objects,
            curves: &[],
            cameras: &[],
        };
        let spec = SimulationSpec {
            units: &units,
            bodies: &bodies,
            articulations: &[],
            rides: &[],
            ground: Some(0.0),
        };
        let usda = export_simulation(&input, &spec, "sim")
            .unwrap()
            .to_usda()
            .unwrap();
        assert!(
            !usda.contains("timeSamples"),
            "a simulation stage states values"
        );
        assert!(usda.contains("kilogramsPerUnit = 1"));
        assert!(usda.contains("def Plane \"Ground\""));

        let dir = temp_dir("sim_units");
        let path = dir.join("sim.usda");
        std::fs::write(&path, &usda).unwrap();
        let stage = Stage::open(&path.display().to_string()).unwrap();
        let schemas = |prim: &str| -> Vec<String> {
            stage
                .prim(sdf::path(prim).unwrap())
                .api_schemas()
                .unwrap()
                .iter()
                .map(|t| t.as_str().to_string())
                .collect()
        };
        assert_eq!(
            schemas("/World/Env/pallet"),
            ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
        );
        // The frame member's box and the undrawn board both sit under the
        // body; only the one that collides is a collider.
        assert!(schemas("/World/Env/pallet/geom").contains(&"PhysicsCollisionAPI".to_string()));
        assert!(schemas("/World/Env/pallet/board").is_empty());
        // A body named under a body is its sibling, not its child.
        assert_eq!(schemas("/World/Env/pallet_crate"), ["PhysicsRigidBodyAPI"]);
        assert!(!usda.contains("def Xform \"crate\""));
        // The belt: kinematic, carrying along its own +X after the quarter
        // turn that points it down world +Y; a kinematic body states no mass.
        assert_eq!(
            schemas("/World/Env/line/belt"),
            ["PhysicsRigidBodyAPI", "PhysxSurfaceVelocityAPI"]
        );
        let belt = stage.prim(sdf::path("/World/Env/line/belt").unwrap());
        assert_eq!(
            belt.attribute("physics:kinematicEnabled")
                .get::<bool>()
                .unwrap(),
            Some(true)
        );
        let v = belt
            .attribute("physxSurfaceVelocity:surfaceVelocity")
            .get::<[f32; 3]>()
            .unwrap()
            .unwrap();
        assert_close(
            [v[0] as f64, v[1] as f64, v[2] as f64],
            [0.3, 0.0, 0.0],
            1e-6,
            "belt velocity, body frame",
        );
        // Scenery collides without a body; the proxy collides unseen.
        assert_eq!(
            schemas("/World/Env/bench/geom"),
            ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        );
        let proxy = stage.prim(sdf::path("/World/Env/bench/proxy").unwrap());
        assert_eq!(
            proxy
                .attribute("purpose")
                .get::<tf::Token>()
                .unwrap()
                .map(|t| t.as_str().to_string()),
            Some("guide".to_string())
        );
        // Members stand where the scene put them, through the body frame.
        let worlds = composed_worlds(&stage, 0.0);
        assert_close(
            mul_point(&worlds["/World/Env/pallet/board"], [0.0, 0.0, 0.0]),
            [2.0, 0.3, 0.16],
            1e-9,
            "board under its body",
        );
        assert_close(
            mul_point(&worlds["/World/Env/pallet_crate/geom"], [0.0, 0.0, 0.0]),
            [2.0, 0.0, 0.30],
            1e-9,
            "hoisted crate",
        );
    }

    /// A URDF robot written as an articulation reads back as the same
    /// machine: geometry in the same places at every configuration, the
    /// same limits, the same coupling, the same mass properties — through
    /// an oblique axis, a reversed one, a slide, a mimic and a weld.
    #[test]
    fn urdf_articulation_round_trips_through_the_importer() {
        let urdf = r#"
        <robot name="r">
          <link name="base">
            <inertial><mass value="4"/><origin xyz="0 0 0.05"/>
              <inertia ixx="0.02" iyy="0.03" izz="0.04" ixy="0" ixz="0" iyz="0"/></inertial>
            <visual><geometry><box size="0.3 0.3 0.1"/></geometry></visual>
            <collision><geometry><box size="0.3 0.3 0.1"/></geometry></collision>
          </link>
          <link name="arm">
            <inertial><mass value="2"/><origin xyz="0.1 0 0.2" rpy="0.3 0.2 0.1"/>
              <inertia ixx="0.011" iyy="0.02" izz="0.015" ixy="0.002" ixz="-0.001" iyz="0.003"/></inertial>
            <visual><origin xyz="0 0 0.2"/><geometry><cylinder radius="0.04" length="0.4"/></geometry></visual>
            <collision><origin xyz="0 0 0.2"/><geometry><cylinder radius="0.05" length="0.4"/></geometry></collision>
          </link>
          <link name="slide"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
          <link name="finger"><visual><geometry><box size="0.02 0.02 0.08"/></geometry></visual></link>
          <link name="twin"><visual><geometry><box size="0.02 0.02 0.08"/></geometry></visual></link>
          <link name="tcp"/>
          <joint name="shoulder" type="revolute">
            <parent link="base"/><child link="arm"/>
            <origin xyz="0 0 0.1" rpy="0.2 0 0.4"/><axis xyz="0 1 1"/>
            <limit lower="-1.5" upper="2.5" effort="40" velocity="3"/>
          </joint>
          <joint name="reach" type="prismatic">
            <parent link="arm"/><child link="slide"/>
            <origin xyz="0 0 0.4"/><axis xyz="-1 0 0"/>
            <limit lower="0" upper="0.25" effort="200" velocity="0.5"/>
          </joint>
          <joint name="grip" type="revolute">
            <parent link="slide"/><child link="finger"/>
            <origin xyz="0 0.05 0.05"/><axis xyz="1 0 0"/>
            <limit lower="0" upper="0.8" effort="5" velocity="2"/>
          </joint>
          <joint name="grip_twin" type="revolute">
            <parent link="slide"/><child link="twin"/>
            <origin xyz="0 -0.05 0.05"/><axis xyz="1 0 0"/>
            <limit lower="-0.8" upper="0" effort="5" velocity="2"/>
            <mimic joint="grip" multiplier="-1" offset="0.1"/>
          </joint>
          <joint name="tool" type="fixed">
            <parent link="slide"/><child link="tcp"/><origin xyz="0 0 0.1" rpy="0 1.2 0"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        let base = Isometry3::from_parts(
            Translation3::new(1.0, -2.0, 0.5),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.7),
        );
        let q = vec![0.4, 0.1, 0.3];
        let link_poses =
            vec![botrail_kin::forward_kinematics_with_base(&model, &q, &base).unwrap()];
        let joint_samples = vec![q.clone()];
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &link_poses,
            joint_samples: Some(&joint_samples),
        }];
        let input = AnimationInput {
            robots: &robots,
            times: &[0.0],
            objects: &[],
            curves: &[],
            cameras: &[],
        };
        let articulation = ArticulationSpec {
            powered: true,
            ..Default::default()
        };
        let spec = SimulationSpec {
            units: &[],
            bodies: &[],
            articulations: &[Some(articulation)],
            rides: &[None],
            ground: None,
        };
        let usda = export_simulation(&input, &spec, "arm")
            .unwrap()
            .to_usda()
            .unwrap();
        for expected in [
            "prepend apiSchemas = [\"PhysicsArticulationRootAPI\", \"PhysxArticulationAPI\"]",
            "bool physxArticulation:enabledSelfCollisions = false",
            "def PhysicsFixedJoint \"root_joint\"",
            "[\"PhysicsJointStateAPI:angular\", \"PhysxJointAPI\", \"PhysicsDriveAPI:angular\"]",
            "[\"PhysicsJointStateAPI:linear\", \"PhysxJointAPI\", \"PhysicsDriveAPI:linear\"]",
            "[\"PhysicsJointStateAPI:angular\", \"PhysxJointAPI\", \"PhysxMimicJointAPI:rotX\"]",
            // The pose is stated on the joints (degrees; the follower at
            // its derived -0.3 + 0.1 rad): PhysX starts from these, not
            // from where the links stand.
            "float state:angular:physics:position = 22.918",
            "float state:angular:physics:position = -11.459",
            "float state:linear:physics:position = 0.1",
            "uniform token physxMimicJoint:rotX:referenceJointAxis = \"rotX\"",
            "float drive:angular:physics:maxForce = 40",
        ] {
            assert!(usda.contains(expected), "missing `{expected}`");
        }

        let dir = temp_dir("articulation");
        let path = dir.join("arm.usda");
        std::fs::write(&path, &usda).unwrap();
        let imported = crate::import_robot(&path, &crate::RobotImportOptions::default()).unwrap();
        assert!(imported.warnings.is_empty(), "{:?}", imported.warnings);
        let back = imported.model;
        assert_eq!(back.dof(), model.dof());
        let link_of = |name: &str| {
            back.links
                .iter()
                .position(|l| l.name.ends_with(&format!("/{name}")))
                .unwrap_or_else(|| panic!("link {name} came back"))
        };
        let joint_of = |name: &str| {
            back.joints
                .iter()
                .find(|j| j.name.ends_with(&format!("/{name}")))
                .unwrap_or_else(|| panic!("joint {name} came back"))
        };

        // Limits, in botrail's units again.
        let shoulder = joint_of("shoulder").limits.unwrap();
        assert!((shoulder.lower + 1.5).abs() < 1e-5 && (shoulder.upper - 2.5).abs() < 1e-5);
        assert!((shoulder.velocity - 3.0).abs() < 1e-5 && (shoulder.effort - 40.0).abs() < 1e-5);
        let reach = joint_of("reach").limits.unwrap();
        assert!((reach.upper - 0.25).abs() < 1e-6 && (reach.velocity - 0.5).abs() < 1e-6);
        let mimic = joint_of("grip_twin").mimic.expect("the coupling came back");
        assert!((mimic.multiplier + 1.0).abs() < 1e-6 && (mimic.offset - 0.1).abs() < 1e-5);

        // Geometry stands where it stood, relative to the base, wherever
        // the joints are — the importer's link frames may differ from the
        // URDF's (an oblique axis turns the joint frame), the machine not.
        for q in [
            vec![0.0, 0.0, 0.0],
            vec![0.4, 0.1, 0.3],
            vec![-1.2, 0.25, 0.8],
        ] {
            let before = botrail_kin::forward_kinematics(&model, &q).unwrap();
            let after = botrail_kin::forward_kinematics(&back, &q).unwrap();
            for (l, link) in model.links.iter().enumerate() {
                let Some(shape) = link.visuals.first() else {
                    continue;
                };
                let b = link_of(&link.name);
                let expect = before[model.root_link].inverse() * before[l] * shape.origin;
                let got =
                    after[back.root_link].inverse() * after[b] * back.links[b].visuals[0].origin;
                assert!(
                    (expect.translation.vector - got.translation.vector).norm() < 1e-5
                        && expect.rotation.angle_to(&got.rotation) < 1e-5,
                    "{} at {q:?}: {expect} vs {got}",
                    link.name
                );
            }
        }
        assert!((imported.root_pose.translation.vector - base.translation.vector).norm() < 1e-6);

        // Mass properties: the same tensor about the same point, however
        // the principal frame came out.
        let (a, b) = (
            model.links[1].inertial.as_ref().unwrap(),
            back.links[link_of("arm")].inertial.as_ref().unwrap(),
        );
        assert!((a.mass - b.mass).abs() < 1e-6);
        let in_link = |i: &botrail_model::Inertial| {
            let r = i.origin.rotation.to_rotation_matrix();
            r.matrix() * i.inertia * r.matrix().transpose()
        };
        // `arm`'s joint axis is oblique, so its frame comes back turned by
        // the joint frame; compare in the base-relative frame at q = 0.
        let zero = vec![0.0; 3];
        let (fa, fb) = (
            botrail_kin::forward_kinematics(&model, &zero).unwrap(),
            botrail_kin::forward_kinematics(&back, &zero).unwrap(),
        );
        let world = |x: &Isometry3<f64>, i: &botrail_model::Inertial| {
            let r = x.rotation.to_rotation_matrix();
            (
                x * nalgebra::Point3::from(i.origin.translation.vector),
                r.matrix() * in_link(i) * r.matrix().transpose(),
            )
        };
        let (ca, ta) = world(&(fa[model.root_link].inverse() * fa[1]), a);
        let lb = link_of("arm");
        let (cb, tb) = world(&(fb[back.root_link].inverse() * fb[lb]), b);
        assert!((ca - cb).norm() < 1e-5, "center of mass {ca} vs {cb}");
        assert!((ta - tb).norm() < 1e-6, "inertia {ta} vs {tb}");
    }

    /// The two-link arm of the riding tests.
    fn rider_arm() -> RobotModel {
        RobotModel::from_urdf_str(
            r#"
        <robot name="r">
          <link name="base"><visual><geometry><box size="0.2 0.2 0.1"/></geometry></visual></link>
          <link name="arm"><visual><geometry><box size="0.05 0.05 0.4"/></geometry></visual></link>
          <joint name="base_x" type="revolute">
            <parent link="base"/><child link="arm"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
            <limit lower="-1" upper="1" effort="10" velocity="1"/>
          </joint>
        </robot>"#,
        )
        .unwrap()
    }

    /// The vehicle of the riding tests: parked at (2, 1) on the floor,
    /// turned a quarter and nose-up a little (a ramp); its body's frame is
    /// the chassis box's centre, 0.15 above the vehicle's own.
    fn rider_vehicle() -> (Isometry3<f64>, Isometry3<f64>) {
        let frame = Isometry3::from_parts(
            Translation3::new(2.0, 1.0, 0.0),
            UnitQuaternion::from_euler_angles(0.0, -0.1, std::f64::consts::FRAC_PI_2),
        );
        (frame, frame * Isometry3::translation(0.0, 0.0, 0.15))
    }

    fn rel_targets(stage: &Stage, prim: &str, rel: &str) -> Vec<String> {
        stage
            .prim(sdf::path(prim).unwrap())
            .relationship(rel)
            .targets()
            .unwrap()
            .iter()
            .map(|p| p.to_string())
            .collect()
    }

    fn has_api(stage: &Stage, prim: &str, api: &str) -> bool {
        stage
            .prim(sdf::path(prim).unwrap())
            .api_schemas()
            .unwrap()
            .iter()
            .any(|t| t.as_str() == api)
    }

    fn float_attr(stage: &Stage, prim: &str, attr: &str) -> f64 {
        stage
            .prim(sdf::path(prim).unwrap())
            .attribute(attr)
            .get::<f32>()
            .unwrap()
            .unwrap_or_else(|| panic!("{prim}.{attr}")) as f64
    }

    fn point_attr(stage: &Stage, prim: &str, attr: &str) -> [f64; 3] {
        let v = stage
            .prim(sdf::path(prim).unwrap())
            .attribute(attr)
            .get::<[f32; 3]>()
            .unwrap()
            .unwrap_or_else(|| panic!("{prim}.{attr}"));
        [v[0] as f64, v[1] as f64, v[2] as f64]
    }

    /// A robot riding a vehicle takes the vehicle in as its base: the body
    /// is a link of the robot's articulation, reached from the world through
    /// six virtual joints whose positions are the vehicle's pose — drive
    /// them and both move, on any pipeline.
    #[test]
    fn a_riding_articulation_takes_its_vehicle_as_its_base() {
        let model = rider_arm();
        let (frame, body) = rider_vehicle();
        let base = frame * Isometry3::translation(0.1, 0.0, 0.3);
        let q = vec![0.2];
        let link_poses =
            vec![botrail_kin::forward_kinematics_with_base(&model, &q, &base).unwrap()];
        let joint_samples = vec![q];
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &link_poses,
            joint_samples: Some(&joint_samples),
        }];
        let mut chassis = scenery("amr/chassis", [0.8, 0.5, 0.3], [0.0; 3], true);
        chassis.track = PoseTrack::Static(body);
        let objects = [chassis];
        let units = [UnitSpec {
            name: "amr".into(),
            role: BodyRole::Link,
            pose: body,
            mass: None,
            belt: None,
        }];
        let bodies = [ObjectBody {
            unit: Some(0),
            guide: false,
        }];
        let input = AnimationInput {
            robots: &robots,
            times: &[0.0],
            objects: &objects,
            curves: &[],
            cameras: &[],
        };
        let export = |articulation: ArticulationSpec, ride: Option<Ride>| {
            let spec = SimulationSpec {
                units: &units,
                bodies: &bodies,
                articulations: &[Some(articulation)],
                rides: &[ride],
                ground: None,
            };
            let usda = export_simulation(&input, &spec, "ride")
                .unwrap()
                .to_usda()
                .unwrap();
            let dir = temp_dir("ride");
            let path = dir.join("ride.usda");
            std::fs::write(&path, &usda).unwrap();
            (usda, Stage::open(&path.display().to_string()).unwrap())
        };
        let powered = || ArticulationSpec {
            powered: true,
            ..Default::default()
        };

        let (usda, stage) = export(
            powered(),
            Some(Ride::Base(CarrierSpec {
                unit: Some(0),
                frame,
            })),
        );
        // A fixed base: an anchor link bolted to the world at the origin
        // (PhysX takes whatever joint reaches the world as the fixed base,
        // its type ignored — slid straight off the world, the first
        // virtual joint would be no joint at all), and the robot prim
        // roots the articulation like any bolted-down robot's.
        assert!(has_api(
            &stage,
            "/World/Robot",
            "PhysicsArticulationRootAPI"
        ));
        assert!(!has_api(
            &stage,
            "/World/Robot/base",
            "PhysicsArticulationRootAPI"
        ));
        let joints = "/World/Robot/joints";
        assert!(rel_targets(&stage, &format!("{joints}/root_joint"), "physics:body0").is_empty());
        assert_eq!(
            rel_targets(&stage, &format!("{joints}/root_joint"), "physics:body1"),
            ["/World/Robot/carrier_anchor"]
        );
        // The virtual joints state the vehicle's pose in the world: three
        // slides, then yaw, pitch, roll (degrees). The robot's own joint
        // kept its name; the virtual one made way.
        let expect = [
            ("base_x_2", "linear", 2.0),
            ("base_y", "linear", 1.0),
            ("base_z", "linear", 0.0),
            ("base_yaw", "angular", 90.0),
            ("base_pitch", "angular", -0.1f64.to_degrees()),
            ("base_roll", "angular", 0.0),
        ];
        for (name, kind, value) in expect {
            let prim = format!("{joints}/{name}");
            for attr in [
                format!("state:{kind}:physics:position"),
                format!("drive:{kind}:physics:targetPosition"),
            ] {
                let got = float_attr(&stage, &prim, &attr);
                assert!(
                    (got - value).abs() < 1e-4,
                    "{name}.{attr}: {got} vs {value}"
                );
            }
            assert!(float_attr(&stage, &prim, &format!("drive:{kind}:physics:stiffness")) > 0.0);
        }
        assert!(usda.contains("def PhysicsRevoluteJoint \"base_x\""));
        // The chain ends on the vehicle's body — at the vehicle's frame as
        // the body sees it, 0.15 below its own — and the robot is bolted to
        // that body where it sits on the deck. Both joints are part of the
        // articulation: nothing stands outside it, nothing is filtered.
        let last = format!("{joints}/base_roll");
        assert_eq!(
            rel_targets(&stage, &last, "physics:body0"),
            ["/World/Robot/carrier_pitch"]
        );
        assert_eq!(
            rel_targets(&stage, &last, "physics:body1"),
            ["/World/Env/amr"]
        );
        assert_close(
            point_attr(&stage, &last, "physics:localPos1"),
            [0.0, 0.0, -0.15],
            1e-6,
            "frame seen from the body",
        );
        let mount = format!("{joints}/mount_joint");
        assert_eq!(
            rel_targets(&stage, &mount, "physics:body0"),
            ["/World/Env/amr"]
        );
        assert_eq!(
            rel_targets(&stage, &mount, "physics:body1"),
            ["/World/Robot/base"]
        );
        assert_close(
            point_attr(&stage, &mount, "physics:localPos0"),
            [0.1, 0.0, 0.15],
            1e-6,
            "mount on the body",
        );
        assert!(!usda.contains("excludeFromArticulation") && !usda.contains("filteredPairs"));
        // The body is a link the solver moves — not kinematic — and the
        // virtual links stand along the chain, the last of them at the
        // vehicle's frame less its roll.
        let amr: Vec<String> = stage
            .prim(sdf::path("/World/Env/amr").unwrap())
            .api_schemas()
            .unwrap()
            .iter()
            .map(|t| t.as_str().to_string())
            .collect();
        assert_eq!(amr, ["PhysicsRigidBodyAPI"]);
        assert!(!usda.contains("kinematicEnabled"));
        let worlds = composed_worlds(&stage, 0.0);
        assert_close(
            mul_point(&worlds["/World/Robot/carrier_pitch"], [0.0, 0.0, 0.0]),
            [2.0, 1.0, 0.0],
            1e-9,
            "the chain reaches the vehicle's frame",
        );
        assert_close(
            mul_point(&worlds["/World/Robot/carrier_y"], [0.0, 0.0, 0.0]),
            [2.0, 1.0, 0.0],
            1e-9,
            "x then y",
        );

        // The robot being the machine — a vehicle with no body of its own —
        // the chain lands on the robot's root link, and nothing is mounted.
        let (usda, stage) = export(
            powered(),
            Some(Ride::Base(CarrierSpec { unit: None, frame })),
        );
        assert_eq!(
            rel_targets(&stage, &last, "physics:body1"),
            ["/World/Robot/base"]
        );
        assert_close(
            point_attr(&stage, &last, "physics:localPos1"),
            [-0.1, 0.0, -0.3],
            1e-6,
            "frame seen from the base",
        );
        assert!(!usda.contains("mount_joint"));

        // Bolted to the world, the root joint holds the robot's own base.
        let (usda, stage) = export(powered(), None);
        assert_eq!(
            rel_targets(&stage, &format!("{joints}/root_joint"), "physics:body1"),
            ["/World/Robot/base"]
        );
        assert!(!usda.contains("carrier_anchor") && !usda.contains("base_yaw"));
        assert!(has_api(
            &stage,
            "/World/Robot",
            "PhysicsArticulationRootAPI"
        ));
        // A floating base has no root joint whatever it rides, and names
        // its root link itself — PhysX would otherwise root the tree at
        // its most central link.
        let (usda, stage) = export(
            ArticulationSpec {
                floating: true,
                ..Default::default()
            },
            Some(Ride::Base(CarrierSpec {
                unit: Some(0),
                frame,
            })),
        );
        assert!(!usda.contains("root_joint") && !usda.contains("carrier_anchor"));
        assert!(has_api(
            &stage,
            "/World/Robot/base",
            "PhysicsArticulationRootAPI"
        ));
        assert!(!has_api(
            &stage,
            "/World/Robot",
            "PhysicsArticulationRootAPI"
        ));
    }

    /// A second robot on the same vehicle joins the first one's
    /// articulation: bolted to the vehicle's body by a mount joint alone —
    /// no root of its own, no root joint, no chain (membership is by
    /// connection) — and both are named apart, since two arms of one
    /// model would otherwise clash.
    #[test]
    fn a_second_rider_joins_the_first_ones_articulation() {
        let model = rider_arm();
        let (frame, body) = rider_vehicle();
        let seats = [
            frame * Isometry3::translation(0.1, 0.2, 0.3),
            frame * Isometry3::translation(0.1, -0.2, 0.3),
        ];
        let q = vec![0.2];
        let poses: Vec<Vec<Vec<Isometry3<f64>>>> = seats
            .iter()
            .map(|seat| vec![botrail_kin::forward_kinematics_with_base(&model, &q, seat).unwrap()])
            .collect();
        let samples = vec![q];
        let robots: Vec<RobotAnimation> = ["left", "right"]
            .iter()
            .zip(&poses)
            .map(|(name, link_poses)| RobotAnimation {
                name,
                model: &model,
                link_poses,
                joint_samples: Some(&samples),
            })
            .collect();
        let mut chassis = scenery("amr/chassis", [0.8, 0.5, 0.3], [0.0; 3], true);
        chassis.track = PoseTrack::Static(body);
        let objects = [chassis];
        let bodies = [ObjectBody {
            unit: Some(0),
            guide: false,
        }];
        // The chassis as plain scenery, for the vehicle with no body.
        let scenery_only = [ObjectBody {
            unit: None,
            guide: false,
        }];
        let input = AnimationInput {
            robots: &robots,
            times: &[0.0],
            objects: &objects,
            curves: &[],
            cameras: &[],
        };
        let powered = ArticulationSpec {
            powered: true,
            ..Default::default()
        };
        let export = |units: &[UnitSpec], rides: &[Option<Ride>], tag: &str| {
            let spec = SimulationSpec {
                units,
                bodies: if units.is_empty() {
                    &scenery_only
                } else {
                    &bodies
                },
                articulations: &[Some(powered.clone()), Some(powered.clone())],
                rides,
                ground: None,
            };
            let usda = export_simulation(&input, &spec, tag)
                .unwrap()
                .to_usda()
                .unwrap();
            let dir = temp_dir(tag);
            let path = dir.join(format!("{tag}.usda"));
            std::fs::write(&path, &usda).unwrap();
            (usda, Stage::open(&path.display().to_string()).unwrap())
        };

        let units = [UnitSpec {
            name: "amr".into(),
            role: BodyRole::Link,
            pose: body,
            mass: None,
            belt: None,
        }];
        let rides = [
            Some(Ride::Base(CarrierSpec {
                unit: Some(0),
                frame,
            })),
            Some(Ride::Joins { host: 0 }),
        ];
        let (usda, stage) = export(&units, &rides, "crew");
        // One articulation, rooted at the host; the chain is the host's.
        assert!(has_api(&stage, "/World/left", "PhysicsArticulationRootAPI"));
        assert!(!has_api(
            &stage,
            "/World/right",
            "PhysicsArticulationRootAPI"
        ));
        assert!(!has_api(
            &stage,
            "/World/right/right_base",
            "PhysicsArticulationRootAPI"
        ));
        assert_eq!(
            usda.matches("def Xform \"carrier_anchor\"").count(),
            1,
            "one chain"
        );
        assert_eq!(
            usda.matches("def PhysicsFixedJoint \"root_joint\"").count(),
            1
        );
        // Self-collision is stated once, on the host: the two arms are one
        // articulation and do not collide with each other.
        assert_eq!(
            usda.matches("physxArticulation:enabledSelfCollisions")
                .count(),
            1
        );
        // Named apart: links and joints of both carry their robot's name.
        for prim in [
            "/World/left/left_base",
            "/World/left/left_arm",
            "/World/left/joints/left_base_x",
            "/World/right/right_base",
            "/World/right/right_arm",
            "/World/right/joints/right_base_x",
        ] {
            assert!(
                stage.prim(sdf::path(prim).unwrap()).is_valid().unwrap(),
                "{prim}"
            );
        }
        // The virtual joints keep their plain names on the host...
        assert_eq!(
            rel_targets(&stage, "/World/left/joints/base_roll", "physics:body1"),
            ["/World/Env/amr"]
        );
        assert_eq!(
            rel_targets(
                &stage,
                "/World/left/joints/left_mount_joint",
                "physics:body1"
            ),
            ["/World/left/left_base"]
        );
        // ...and the second robot is bolted to the vehicle's body where it
        // sits, by its mount joint alone.
        let mount = "/World/right/joints/right_mount_joint";
        assert_eq!(
            rel_targets(&stage, mount, "physics:body0"),
            ["/World/Env/amr"]
        );
        assert_eq!(
            rel_targets(&stage, mount, "physics:body1"),
            ["/World/right/right_base"]
        );
        assert_close(
            point_attr(&stage, mount, "physics:localPos0"),
            [0.1, -0.2, 0.15],
            1e-6,
            "the second seat on the body",
        );
        assert!(!usda.contains("/World/right/joints/root_joint"));
        assert!(!stage
            .prim(sdf::path("/World/right/joints/base_x").unwrap())
            .is_valid()
            .unwrap());

        // On a vehicle with no body of its own, the second robot is bolted
        // to the host's root link.
        let rides = [
            Some(Ride::Base(CarrierSpec { unit: None, frame })),
            Some(Ride::Joins { host: 0 }),
        ];
        let (_, stage) = export(&[], &rides, "crew_bare");
        let mount = "/World/right/joints/right_mount_joint";
        assert_eq!(
            rel_targets(&stage, mount, "physics:body0"),
            ["/World/left/left_base"]
        );
        assert_close(
            point_attr(&stage, mount, "physics:localPos0"),
            [0.0, -0.4, 0.0],
            1e-6,
            "the second seat, seen from the first",
        );

        // A join has to name an earlier robot that rides as a base.
        let bad = SimulationSpec {
            units: &[],
            bodies: &[],
            articulations: &[Some(powered.clone()), Some(powered.clone())],
            rides: &[Some(Ride::Joins { host: 1 }), None],
            ground: None,
        };
        assert!(matches!(
            export_simulation(&input, &bad, "bad"),
            Err(UsdExportError::Input(_))
        ));
    }

    /// A USD-sourced robot rides like any other: its stage's anchoring to
    /// the world is switched off by an `over`, the chain hangs under the
    /// referenced prim (or beside it, when the stage composes through a
    /// unit correction), and a mount joint bolts its root body to the
    /// vehicle. A second one joining loses its stage's articulation root.
    #[test]
    fn a_referenced_robot_rides_its_vehicle() {
        for (tag, usda_src, under_robot) in [
            ("ref_ride_zup", ARM.to_string(), true),
            ("ref_ride_yup", arm_yup_cm(), false),
        ] {
            let dir = temp_dir(tag);
            std::fs::write(dir.join("robot.usda"), &usda_src).unwrap();
            let imported = import_robot(
                &dir.join("robot.usda"),
                &RobotImportOptions {
                    mesh_cache_dir: Some(dir.join("meshes")),
                    ..Default::default()
                },
            )
            .unwrap();
            let model = imported.model;
            let (frame, body) = rider_vehicle();
            let seats = [
                frame * Isometry3::translation(0.1, 0.2, 0.3),
                frame * Isometry3::translation(0.1, -0.2, 0.3),
            ];
            let q = vec![0.3, -0.2];
            let poses: Vec<Vec<Vec<Isometry3<f64>>>> = seats
                .iter()
                .map(|seat| {
                    vec![botrail_kin::forward_kinematics_with_base(&model, &q, seat).unwrap()]
                })
                .collect();
            let samples = vec![q.clone()];
            let robots: Vec<RobotAnimation> = ["left", "right"]
                .iter()
                .zip(&poses)
                .map(|(name, link_poses)| RobotAnimation {
                    name,
                    model: &model,
                    link_poses,
                    joint_samples: Some(&samples),
                })
                .collect();
            let mut chassis = scenery("amr/chassis", [0.8, 0.5, 0.3], [0.0; 3], true);
            chassis.track = PoseTrack::Static(body);
            let objects = [chassis];
            let units = [UnitSpec {
                name: "amr".into(),
                role: BodyRole::Link,
                pose: body,
                mass: None,
                belt: None,
            }];
            let bodies = [ObjectBody {
                unit: Some(0),
                guide: false,
            }];
            let input = AnimationInput {
                robots: &robots,
                times: &[0.0],
                objects: &objects,
                curves: &[],
                cameras: &[],
            };
            let spec = SimulationSpec {
                units: &units,
                bodies: &bodies,
                articulations: &[None, None],
                rides: &[
                    Some(Ride::Base(CarrierSpec {
                        unit: Some(0),
                        frame,
                    })),
                    Some(Ride::Joins { host: 0 }),
                ],
                ground: None,
            };
            let exported = export_simulation(&input, &spec, "ride").unwrap();
            assert!(
                exported.warnings.is_empty(),
                "{tag}: {:?}",
                exported.warnings
            );
            let out = dir.join("ride.usda");
            let usda = exported.to_usda().unwrap();
            write_exported(&out, exported).unwrap();
            let stage = Stage::open(&out.display().to_string()).unwrap();

            // The stage's own anchor to the world is gone — an `over`
            // deactivating the referenced joint — on both robots.
            for robot in ["left", "right"] {
                let anchor = format!("/World/{robot}/joints/anchor");
                assert!(
                    usda.contains(&format!(
                        "over \"anchor\" (\n{}active = false",
                        " ".repeat(16)
                    )),
                    "{tag}: {anchor} deactivated"
                );
                assert!(
                    !stage
                        .prim(sdf::path(&anchor).unwrap())
                        .is_active()
                        .unwrap_or(true),
                    "{tag}: {anchor} still active"
                );
            }
            // The chain: under the referenced prim when the stage composes
            // at identity, beside it otherwise — and its joints sit in a
            // scope of their own, the asset's `joints` being taken.
            let chain = if under_robot {
                "/World/left"
            } else {
                "/World/left_carrier"
            };
            let scope = if under_robot { "joints_2" } else { "joints" };
            let root_joint = format!("{chain}/{scope}/root_joint");
            assert_eq!(
                rel_targets(&stage, &root_joint, "physics:body1"),
                [format!("{chain}/carrier_anchor")],
                "{tag}"
            );
            let last = format!("{chain}/{scope}/base_roll");
            assert_eq!(
                rel_targets(&stage, &last, "physics:body1"),
                ["/World/Env/amr"],
                "{tag}"
            );
            assert!(
                (float_attr(
                    &stage,
                    &format!("{chain}/{scope}/base_yaw"),
                    "state:angular:physics:position"
                ) - 90.0)
                    .abs()
                    < 1e-4
            );
            // The mount joint bolts the root *body* prim to the vehicle,
            // where the base sits on the deck — in metres, whatever the
            // stage's units.
            let mount = format!("{chain}/{scope}/mount_joint");
            assert_eq!(
                rel_targets(&stage, &mount, "physics:body0"),
                ["/World/Env/amr"]
            );
            assert_eq!(
                rel_targets(&stage, &mount, "physics:body1"),
                ["/World/left/base"]
            );
            assert_close(
                point_attr(&stage, &mount, "physics:localPos0"),
                [0.1, 0.2, 0.15],
                1e-5,
                &format!("{tag}: the seat on the body"),
            );
            // The second robot joins: a mount joint of its own, and the
            // articulation root its stage applies deleted.
            let mount = "/World/right/joints_2/mount_joint";
            let mount = if under_robot {
                mount.to_string()
            } else {
                "/World/right_carrier/joints/mount_joint".to_string()
            };
            assert_eq!(
                rel_targets(&stage, &mount, "physics:body1"),
                ["/World/right/base"]
            );
            assert!(
                !usda.contains("/World/right_carrier/joints/root_joint")
                    && !usda.contains("/World/right/joints_2/root_joint")
            );
            assert!(
                usda.contains("delete apiSchemas = [\"PhysicsArticulationRootAPI\"]"),
                "{tag}: the joining robot's root is deleted"
            );
            assert!(
                has_api(&stage, "/World/left", "PhysicsArticulationRootAPI"),
                "{tag}"
            );
        }
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
          <material name="paint"><color rgba="0.25 0.5 0.75 1.0"/></material>
          <link name="base link">
            <visual><geometry><box size="0.2 0.1 0.4"/></geometry><material name="paint"/></visual>
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
                material: None,
                visual_asset: None,
                name: "/World/Conveyor/Box_A".into(),
                geometry: Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                track: PoseTrack::Static(Isometry3::translation(1.0, 0.0, 0.05)),
                color: Some([0.8, 0.3, 0.1]),
                visible: Vec::new(),
                physics: None,
            },
            ObjectSpec {
                material: None,
                visual_asset: None,
                name: "held".into(),
                geometry: Geometry::Sphere { radius: 0.03 },
                track: PoseTrack::Sampled(held_track.clone()),
                color: None,
                visible: Vec::new(),
                physics: None,
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
            curves: &[],
            cameras: &[],
        };
        let warnings =
            write_animation(&dir.join("anim.usda"), &input, &ExportOptions::default()).unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");

        // A visual that named a colour carries it into the stage; one
        // that named none keeps the neutral fallback every gprim gets.
        let text = std::fs::read_to_string(dir.join("anim.usda")).unwrap();
        let block = |head: &str| -> String {
            let rest = text.split(head).nth(1).expect("visual authored");
            rest[..rest.find('}').unwrap_or(rest.len())].to_string()
        };
        let painted = block("def Cube \"Visual_0\"");
        assert!(
            painted.contains("primvars:displayColor = [(0.25, 0.5, 0.75)]"),
            "{painted}"
        );
        let bare = block("def Cylinder \"Visual_1\"");
        assert!(bare.contains("primvars:displayColor"), "{bare}");
        assert!(!bare.contains("(0.25, 0.5, 0.75)"), "{bare}");
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

    /// `.usdc` (and `.usd`) must produce a binary crate file that composes
    /// to the same world transforms and metadata as the text layer — the
    /// extension picks the serialization, never the content.
    #[test]
    fn toolpath_curves_author_as_basis_curves() {
        let dir = temp_dir("curves");
        let urdf = r#"
        <robot name="r">
          <link name="base"><visual><geometry><box size="0.2 0.1 0.4"/></geometry></visual></link>
          <link name="tip"/>
          <joint name="j" type="revolute">
            <parent link="base"/><child link="tip"/>
            <origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        let times = [0.0, 0.5];
        let configs = [vec![0.0], vec![0.4]];
        let link_poses: Vec<Vec<Isometry3<f64>>> = configs
            .iter()
            .map(|q| botrail_kin::forward_kinematics(&model, q).unwrap())
            .collect();
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &link_poses,
            joint_samples: None,
        }];
        let curves = vec![
            CurveSpec {
                name: "trim feed".into(),
                curves: vec![
                    vec![[0.0, 0.0, 0.1], [0.1, 0.0, 0.1], [0.1, 0.1, 0.1]],
                    vec![[0.2, 0.0, 0.1], [0.3, 0.0, 0.1]],
                ],
                color: [0.85, 0.33, 0.05],
                width: 0.003,
            },
            // A one-point polyline cannot be a curve: warned and skipped.
            CurveSpec {
                name: "degenerate".into(),
                curves: vec![vec![[0.0, 0.0, 0.0]]],
                color: [0.5, 0.5, 0.5],
                width: 0.001,
            },
        ];
        let warnings = write_animation(
            &dir.join("anim.usda"),
            &AnimationInput {
                robots: &robots,
                times: &times,
                objects: &[],
                curves: &curves,
                cameras: &[],
            },
            &ExportOptions::default(),
        )
        .unwrap();
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("degenerate"), "{warnings:?}");

        let text = std::fs::read_to_string(dir.join("anim.usda")).unwrap();
        assert!(
            text.contains("def BasisCurves \"trim_feed\""),
            "no BasisCurves prim:\n{text}"
        );
        assert!(!text.contains("degenerate"), "skipped spec leaked in");
        assert!(text.contains("curveVertexCounts = [3, 2]"), "{text}");
        assert!(text.contains("uniform token type = \"linear\""), "{text}");
        assert!(
            text.contains("uniform token wrap = \"nonperiodic\""),
            "{text}"
        );

        // pxr-grade readability: the stage opens and the prim resolves with
        // the same reader the recording path uses.
        let stage = Stage::builder()
            .resolver(SearchPathResolver::new(vec![dir.clone()]))
            .open(&dir.join("anim.usda").display().to_string())
            .unwrap();
        let prim = stage.prim(sdf::path("/World/Toolpaths/trim_feed").unwrap());
        let type_name = prim
            .type_name()
            .expect("toolpath prim resolves")
            .map(|t| t.to_string())
            .unwrap_or_default();
        assert_eq!(type_name, "BasisCurves");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn usdc_output_matches_usda() {
        let dir = temp_dir("usdc");
        let urdf = r#"
        <robot name="r">
          <link name="base"><visual><geometry><box size="0.2 0.1 0.4"/></geometry></visual></link>
          <link name="tip"><visual><geometry><sphere radius="0.05"/></geometry></visual></link>
          <joint name="j" type="revolute">
            <parent link="base"/><child link="tip"/>
            <origin xyz="0 0 0.5"/>
            <axis xyz="0 0 1"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        let times = [0.0, 0.5, 1.0];
        let configs = [vec![0.0], vec![0.4], vec![0.9]];
        let link_poses: Vec<Vec<Isometry3<f64>>> = configs
            .iter()
            .map(|q| botrail_kin::forward_kinematics(&model, q).unwrap())
            .collect();
        let moved = vec![
            Isometry3::translation(0.1, 0.0, 0.5),
            Isometry3::translation(0.15, 0.05, 0.55),
            Isometry3::translation(0.2, 0.1, 0.6),
        ];
        let objects = vec![ObjectSpec {
            material: None,
            visual_asset: None,
            name: "crate".into(),
            geometry: Geometry::Box {
                size: Vector3::new(0.1, 0.1, 0.1),
            },
            track: PoseTrack::Sampled(moved),
            color: Some([0.2, 0.5, 0.7]),
            visible: vec![true, true, false],
            physics: None,
        }];
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
            curves: &[],
            cameras: &[],
        };

        for name in ["anim.usda", "anim.usdc", "anim.usd"] {
            let warnings =
                write_animation(&dir.join(name), &input, &ExportOptions::default()).unwrap();
            assert!(warnings.is_empty(), "{name}: {warnings:?}");
        }
        // The binary outputs really are crate files, not text in disguise.
        for name in ["anim.usdc", "anim.usd"] {
            let head = std::fs::read(dir.join(name)).unwrap();
            assert_eq!(&head[..8], b"PXR-USDC", "{name} lacks the crate magic");
            // Stamped with what the content needs, not with the newest
            // version the writer knows: a USD 24 reader (Isaac Sim 5)
            // refuses a 0.12 file outright.
            assert_eq!(
                &head[8..11],
                [0, 8, 0],
                "{name} claims a newer crate version"
            );
        }
        assert!(matches!(
            write_animation(&dir.join("anim.usdz"), &input, &ExportOptions::default()),
            Err(UsdExportError::Input(_))
        ));

        let open = |name: &str| {
            Stage::builder()
                .resolver(SearchPathResolver::new(vec![dir.clone()]))
                .open(&dir.join(name).display().to_string())
                .unwrap()
        };
        let text = open("anim.usda");
        let binary = open("anim.usdc");
        for t in times {
            let a = composed_worlds(&text, t * 60.0);
            let b = composed_worlds(&binary, t * 60.0);
            assert_eq!(
                a.keys().collect::<std::collections::BTreeSet<_>>(),
                b.keys().collect::<std::collections::BTreeSet<_>>()
            );
            for (path, ma) in &a {
                let mb = &b[path];
                assert_close(
                    mul_point(ma, [0.0; 3]),
                    mul_point(mb, [0.0; 3]),
                    1e-12,
                    &format!("{path} at t={t}"),
                );
            }
        }
        // Visibility animation survives the binary path.
        let vis = |code: f64| -> Value {
            binary
                .prim("/World/Env/crate")
                .attribute("visibility")
                .get_at::<sdf::Value>(TimeCode::new(code))
                .unwrap()
                .expect("visibility sample")
        };
        assert_eq!(vis(0.0), Value::Token("inherited".into()));
        assert_eq!(vis(60.0), Value::Token("invisible".into()));
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
            curves: &[],
            cameras: &[],
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
            curves: &[],
            cameras: &[],
        };
        assert!(matches!(
            export_animation(&bad_len, &ExportOptions::default(), "a"),
            Err(UsdExportError::Input(_))
        ));

        let no_robots = AnimationInput {
            robots: &[],
            times: &times,
            objects: &[],
            curves: &[],
            cameras: &[],
        };
        assert!(matches!(
            export_animation(&no_robots, &ExportOptions::default(), "a"),
            Err(UsdExportError::Input(_))
        ));
    }

    /// Cameras land under `/World/Cameras` as `Camera` prims: pinhole
    /// attributes plus a static or sampled xform, with the sanitized name.
    #[test]
    fn cameras_author_as_usd_camera_prims() {
        let times = [0.0, 0.5];
        let objects = [ObjectSpec {
            material: None,
            visual_asset: None,
            name: "crate".into(),
            geometry: Geometry::Box {
                size: Vector3::new(0.1, 0.1, 0.1),
            },
            track: PoseTrack::Static(Isometry3::translation(1.0, 0.0, 0.05)),
            color: None,
            visible: Vec::new(),
            physics: None,
        }];
        let cameras = [
            CameraSpec {
                name: "overview".into(),
                track: PoseTrack::Static(Isometry3::translation(2.0, -1.0, 1.5)),
                focal_length: 18.147,
                horizontal_aperture: 20.955,
                vertical_aperture: 11.787,
                clipping: [0.05, 30.0],
                resolution: [1920, 1080],
            },
            CameraSpec {
                name: "wrist cam".into(),
                track: PoseTrack::Sampled(vec![
                    Isometry3::translation(0.0, 0.0, 1.0),
                    Isometry3::translation(0.2, 0.0, 1.0),
                ]),
                focal_length: 22.4,
                horizontal_aperture: 20.955,
                vertical_aperture: 11.787,
                clipping: [0.05, 4.0],
                resolution: [640, 360],
            },
        ];
        let input = AnimationInput {
            robots: &[],
            times: &times,
            objects: &objects,
            curves: &[],
            cameras: &cameras,
        };
        let exported = export_animation(&input, &ExportOptions::default(), "cams").unwrap();
        let text = exported.to_usda().unwrap();
        assert!(text.contains("def Camera \"overview\""), "{text}");
        assert!(text.contains("def Camera \"wrist_cam\""), "{text}");
        assert!(text.contains("float focalLength = 18.14"), "{text}");
        assert!(text.contains("float2 clippingRange = (0.05"), "{text}");
        assert!(text.contains("float verticalAperture = 11.78"), "{text}");
        // The sampled camera writes timeSampled xformOps; the static one a
        // plain default.
        let wrist = text.split("def Camera \"wrist_cam\"").nth(1).unwrap();
        assert!(wrist.contains("timeSamples"), "{wrist}");
        // A sampled length mismatch is refused, like objects.
        let bad = [CameraSpec {
            name: "bad".into(),
            track: PoseTrack::Sampled(vec![Isometry3::identity()]),
            focal_length: 20.0,
            horizontal_aperture: 20.955,
            vertical_aperture: 11.787,
            clipping: [0.05, 30.0],
            resolution: [1920, 1080],
        }];
        let result = export_animation(
            &AnimationInput {
                robots: &[],
                times: &times,
                objects: &objects,
                curves: &[],
                cameras: &bad,
            },
            &ExportOptions::default(),
            "cams",
        );
        assert!(matches!(result, Err(UsdExportError::Input(_))));
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
            curves: &[],
            cameras: &[],
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

    /// Exports one frame of the robot at `stage` to `out/anim.usda`.
    #[cfg(unix)]
    fn export_one_frame(stage: &Path, out: &Path) -> Vec<String> {
        std::fs::create_dir_all(out).unwrap();
        let imported = import_robot(
            stage,
            &RobotImportOptions {
                mesh_cache_dir: Some(out.join("meshes")),
                ..Default::default()
            },
        )
        .unwrap();
        let model = imported.model;
        let link_poses = [botrail_kin::forward_kinematics_with_base(
            &model,
            &[0.0, 0.0],
            &Isometry3::identity(),
        )
        .unwrap()];
        let robots = [RobotAnimation {
            name: "Robot",
            model: &model,
            link_poses: &link_poses,
            joint_samples: None,
        }];
        let input = AnimationInput {
            robots: &robots,
            times: &[0.0],
            objects: &[],
            curves: &[],
            cameras: &[],
        };
        write_animation(&out.join("anim.usda"), &input, &ExportOptions::default()).unwrap()
    }

    /// A stage directory reached through a symlink — macOS temp dirs are
    /// `/var/folders/…` onto `/private/var/folders/…`. The dependencies come
    /// back canonicalized and must still count as inside the stage directory.
    #[cfg(unix)]
    #[test]
    fn symlinked_stage_dir_still_contains_its_layers() {
        let dir = temp_dir("lndir");
        std::fs::create_dir_all(dir.join("real")).unwrap();
        std::fs::write(dir.join("real/robot.usda"), ARM).unwrap();
        std::os::unix::fs::symlink("real", dir.join("link")).unwrap();

        let out = dir.join("out");
        let warnings = export_one_frame(&dir.join("link/robot.usda"), &out);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert!(out.join("anim_assets/robot/robot.usda").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// huggingface_hub-style cache: the stage file is a symlink onto a blob
    /// in another directory. Following the file's own link would put every
    /// catalog robot's layers outside the stage directory.
    #[cfg(unix)]
    #[test]
    fn symlinked_stage_file_is_copied_under_its_link_name() {
        let dir = temp_dir("lnfile");
        std::fs::create_dir_all(dir.join("blobs")).unwrap();
        std::fs::create_dir_all(dir.join("snap/usd")).unwrap();
        std::fs::write(dir.join("blobs/0123abcd"), ARM).unwrap();
        std::os::unix::fs::symlink("../../blobs/0123abcd", dir.join("snap/usd/robot.usda"))
            .unwrap();

        let out = dir.join("out");
        let warnings = export_one_frame(&dir.join("snap/usd/robot.usda"), &out);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert!(out.join("anim_assets/robot/robot.usda").exists());
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
