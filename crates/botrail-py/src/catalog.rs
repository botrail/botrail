//! `Robot.from_catalog`: loading packages from the botrail model catalog
//! (the Hugging Face dataset `botrail/botrail-catalog`, built by
//! botrail-catalog-builder).
//!
//! The heavy lifting — downloads, caching, auth, revision resolution — is
//! delegated to the optional Python dependency `huggingface_hub`
//! (`pip install botrail[catalog]`); this module only orchestrates it:
//!
//! 1. `dataset_info(...).sha` pins the revision (a floating "newest"
//!    becomes a concrete commit, which is what projects record).
//! 2. `index.json` resolves the product id — exact, or by segment
//!    subsequence, so `robotiq/2f-85` and `2f-85` both find
//!    `robotiq/2f/2f-85/r1` as long as they are unambiguous.
//! 3. `snapshot_download(allow_patterns=[<id>/*])` fetches the package;
//!    `manifest.yaml` supplies the model asset paths and
//!    `frames.tcp_default`.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use botrail_model::{CatalogArm, CatalogMeta, RobotModel, RobotSource};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyAnyMethods, PyDict};

const REPO_ID: &str = "botrail/botrail-catalog";
const REPO_URL: &str = "https://huggingface.co/datasets/botrail/botrail-catalog";

fn err(message: String) -> PyErr {
    PyValueError::new_err(message)
}

/// One `index.json` product entry, reduced to what loading needs.
struct IndexEntry {
    id: String,
    distribution: String,
    urdf: Option<String>,
    usd: Option<String>,
}

/// The catalog package directory for `query`, downloaded whole.
///
/// Not every catalog package is a robot. A body-in-white is a pile of
/// collision meshes that a cell loads as obstacles, and a fixture is a
/// mesh plus a frame — both want the package on disk, not a `RobotModel`.
/// Returning the directory keeps those callers off `huggingface_hub`
/// internals and, more to the point, off a hand-written snapshot path
/// that silently stops matching when the dataset moves.
#[pyfunction]
#[pyo3(signature = (id, *, revision=None))]
pub fn catalog_package(py: Python<'_>, id: &str, revision: Option<&str>) -> PyResult<String> {
    let (snapshot, entry, _sha) = download_package(py, id, revision)?;
    Ok(snapshot.join(&entry.id).to_string_lossy().into_owned())
}

/// Resolve `query` to a package and fetch the whole thing, returning the
/// snapshot root (package paths in the index are repo-relative to it).
fn download_package(
    py: Python<'_>,
    query: &str,
    revision: Option<&str>,
) -> PyResult<(PathBuf, IndexEntry, String)> {
    let hub = py.import("huggingface_hub").map_err(|_| {
        err(
            "the catalog needs the optional dependency `huggingface_hub` — install it \
             with `pip install botrail[catalog]`"
                .to_string(),
        )
    })?;

    // Pin the revision to a commit SHA before any download, so every file
    // comes from one consistent snapshot and projects can replay it.
    // `dataset_info` implies the repo type and (unlike the download calls)
    // takes no `repo_type` argument on huggingface_hub 1.x — do not pass it.
    let kwargs = PyDict::new(py);
    kwargs.set_item("revision", revision)?;
    let info = hub
        .call_method("dataset_info", (REPO_ID,), Some(&kwargs))
        .map_err(|e| err(format!("cannot reach the catalog dataset {REPO_URL}: {e}")))?;
    let sha: String = info.getattr("sha")?.extract()?;

    let entry = resolve_entry(py, &hub, &sha, query)?;
    if entry.distribution != "public" {
        return Err(err(format!(
            "catalog package `{}` is distributed as `{}`: it ships metadata only. Build the \
             package locally with botrail-catalog-builder and load the result with \
             Robot.from_urdf / Robot.from_usd (see {REPO_URL})",
            entry.id, entry.distribution
        )));
    }

    // Fetch the whole package directory: a URDF references its meshes by
    // relative path, so per-file downloads would tear the package apart.
    let kwargs = PyDict::new(py);
    kwargs.set_item("repo_type", "dataset")?;
    kwargs.set_item("revision", &sha)?;
    kwargs.set_item("allow_patterns", vec![format!("{}/*", entry.id)])?;
    let snapshot: String = hub
        .call_method("snapshot_download", (REPO_ID,), Some(&kwargs))
        .map_err(|e| err(format!("catalog download failed for `{}`: {e}", entry.id)))?
        .extract()?;
    Ok((PathBuf::from(snapshot), entry, sha))
}

pub fn from_catalog(
    py: Python<'_>,
    query: &str,
    revision: Option<&str>,
    format: Option<&str>,
) -> PyResult<RobotModel> {
    match format {
        None | Some("urdf") | Some("usd") => {}
        Some(other) => {
            return Err(err(format!(
                "unknown format `{other}`; pass \"urdf\", \"usd\", or omit it"
            )))
        }
    }
    let (snapshot, entry, sha) = download_package(py, query, revision)?;
    let package_dir = snapshot.join(&entry.id);

    let manifest = read_manifest(py, &package_dir)?;

    // Prefer the URDF (meshes resolve as plain relative paths); the USD is
    // authoritative when asked for or when it is all the package ships.
    let (rel, is_usd) = match format {
        Some("usd") => (entry.usd.clone(), true),
        Some(_) => (entry.urdf.clone(), false),
        None => match (&entry.urdf, &entry.usd) {
            (Some(urdf), _) => (Some(urdf.clone()), false),
            (None, Some(usd)) => (Some(usd.clone()), true),
            (None, None) => (None, false),
        },
    };
    let Some(rel) = rel else {
        return Err(err(format!(
            "catalog package `{}` ships no {} model",
            entry.id,
            format.unwrap_or("urdf or usd")
        )));
    };
    // Index asset paths are repo-relative (`<id>/<rel>`).
    let model_path = snapshot.join(&rel);

    let mut model = if is_usd {
        let imported =
            botrail_usd::import_robot(&model_path, &botrail_usd::RobotImportOptions::default())
                .map_err(|e| err(format!("{}: {e}", model_path.display())))?;
        for warning in &imported.warnings {
            eprintln!("botrail: catalog `{}`: {warning}", entry.id);
        }
        imported.model
    } else {
        RobotModel::from_urdf_file(&model_path)
            .map_err(|e| err(format!("{}: {e}", model_path.display())))?
    };

    // The manifest's declared frames beat the heuristics: tcp_default
    // replaces the deepest-leaf guess, flange/mount give `attach_tool` its
    // argument-free defaults. USD link names are prim paths; match by last
    // path segment there.
    let resolve = |field: &str, name: Option<&str>| -> Option<usize> {
        let name = name?;
        let index = model.link_index(name).or_else(|| {
            let matches: Vec<usize> = (0..model.links.len())
                .filter(|&i| model.links[i].name.rsplit('/').next() == Some(name))
                .collect();
            match matches.as_slice() {
                [one] => Some(*one),
                _ => None,
            }
        });
        if index.is_none() {
            eprintln!(
                "botrail: catalog `{}`: manifest {field} `{name}` is not a link; ignored",
                entry.id
            );
        }
        index
    };
    let tcp = resolve("tcp_default", manifest.tcp_default.as_deref());
    let flange = resolve("flange_frame", manifest.flange_frame.as_deref());
    let mount = resolve("mount_frame", manifest.mount_frame.as_deref());
    let grasp: Vec<usize> = manifest
        .grasp_frames
        .iter()
        .filter_map(|f| resolve("grasp_frames", Some(f)))
        .collect();
    // A dual-arm package names its arms: each becomes a planning group,
    // tipped at the arm's TCP, driving the joints the manifest lists.
    // Resolved here, while the name lookup is alive; declared below.
    let mut resolved_arms = Vec::new();
    for arm in &manifest.arms {
        let tip = resolve("arms[].tcp_default", Some(&arm.tcp_default)).ok_or_else(|| {
            err(format!(
                "catalog `{}`: arm `{}` tips at `{}`, which is not a link",
                entry.id, arm.name, arm.tcp_default
            ))
        })?;
        let flange = arm
            .flange_frame
            .as_deref()
            .and_then(|f| resolve("arms[].flange_frame", Some(f)));
        resolved_arms.push((arm.name.clone(), tip, flange, arm.joints.clone()));
    }
    model.tcp_link = tcp;
    model.flange_link = flange;
    model.mount_link = mount;
    model.grasp_links = grasp;

    let mut arms = Vec::new();
    for (name, tip, flange, joints) in resolved_arms {
        let tip = model.links[tip].name.clone();
        let flange = flange.map(|i| model.links[i].name.clone());
        let joint_refs: Vec<&str> = joints.iter().map(String::as_str).collect();
        model = model
            .define_group(
                &name,
                &tip,
                (!joint_refs.is_empty()).then_some(joint_refs.as_slice()),
                flange.as_deref(),
            )
            .map_err(|e| err(format!("catalog `{}`: arm `{name}`: {e}", entry.id)))?;
        arms.push(CatalogArm {
            name,
            tip,
            joints,
            flange,
        });
    }

    let link_name = |i: usize| model.links[i].name.clone();
    let grasp_names = model.grasp_links.iter().map(|&i| link_name(i)).collect();
    let inner = std::mem::replace(&mut model.source, RobotSource::UrdfXml(String::new()));
    model.source = RobotSource::Catalog {
        id: entry.id,
        revision: sha,
        arms,
        tcp: tcp.map(link_name),
        flange: flange.map(link_name),
        mount: mount.map(link_name),
        grasp: grasp_names,
        meta: manifest.meta,
        inner: Box::new(inner),
    };
    Ok(model)
}

/// Downloads `index.json` at the pinned revision and resolves `query` to a
/// product entry: an exact id, or a unique match whose id path segments
/// contain the query's segments as a subsequence.
fn resolve_entry(
    py: Python<'_>,
    hub: &Bound<'_, PyAny>,
    sha: &str,
    query: &str,
) -> PyResult<IndexEntry> {
    let kwargs = PyDict::new(py);
    kwargs.set_item("repo_type", "dataset")?;
    kwargs.set_item("revision", sha)?;
    kwargs.set_item("filename", "index.json")?;
    let index_path: String = hub
        .call_method("hf_hub_download", (REPO_ID,), Some(&kwargs))
        .map_err(|e| {
            err(format!(
                "cannot fetch the catalog index from {REPO_URL}: {e}"
            ))
        })?
        .extract()?;
    let index: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(PathBuf::from(&index_path))
            .map_err(|e| err(format!("{index_path}: {e}")))?,
    )
    .map_err(|e| err(format!("{index_path}: invalid index.json: {e}")))?;

    let entries: Vec<IndexEntry> = index["products"]
        .as_array()
        .map(|products| {
            products
                .iter()
                .filter_map(|p| {
                    Some(IndexEntry {
                        id: p["id"].as_str()?.to_string(),
                        distribution: p["distribution"].as_str().unwrap_or("public").to_string(),
                        urdf: p["assets"]["urdf"].as_str().map(str::to_string),
                        usd: p["assets"]["usd"].as_str().map(str::to_string),
                    })
                })
                .collect()
        })
        .unwrap_or_default();

    if let Some(exact) = entries.iter().position(|e| e.id == query) {
        return Ok(entries.into_iter().nth(exact).expect("position is valid"));
    }
    let want: Vec<&str> = query.split('/').filter(|s| !s.is_empty()).collect();
    let is_subsequence = |id: &str| {
        let mut want = want.iter().peekable();
        for segment in id.split('/') {
            if want.peek() == Some(&&segment) {
                want.next();
            }
        }
        want.peek().is_none()
    };
    let mut matches: Vec<usize> = entries
        .iter()
        .enumerate()
        .filter(|(_, e)| is_subsequence(&e.id))
        .map(|(i, _)| i)
        .collect();
    // Several matches that differ only in the trailing revision segment are
    // one product, re-cut: `.../r2000ic-165f/r1` and `.../r2` are the same
    // machine from a better source. Take the newest rather than making
    // every catalog revision a breaking change for anyone naming the
    // product by its short name. This does not loosen reproducibility —
    // the resolved id and the dataset SHA are what projects and generated
    // scripts record, so a replay stays on the revision it resolved to.
    if matches.len() > 1 {
        if let Some(newest) = newest_revision(&entries, &matches) {
            matches = vec![newest];
        }
    }
    match matches.as_slice() {
        [one] => Ok(entries.into_iter().nth(*one).expect("index is valid")),
        [] => Err(err(format!(
            "`{query}` is not in the catalog. Available: {} (see {REPO_URL})",
            entries
                .iter()
                .map(|e| e.id.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ))),
        many => Err(err(format!(
            "`{query}` is ambiguous in the catalog: matches {}",
            many.iter()
                .map(|&i| entries[i].id.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ))),
    }
}

/// The highest-revision entry when `matches` are revisions of one product —
/// identical ids but for a trailing `r<N>` segment. `None` when they are
/// genuinely different products (or carry a revision this cannot order),
/// which stays an ambiguity for the caller to resolve.
fn newest_revision(entries: &[IndexEntry], matches: &[usize]) -> Option<usize> {
    let split = |i: usize| -> Option<(&str, u32)> {
        let id = entries[i].id.as_str();
        let (product, rev) = id.rsplit_once('/')?;
        let n = rev.strip_prefix('r')?.parse::<u32>().ok()?;
        Some((product, n))
    };
    let (product, _) = split(matches[0])?;
    let mut best = (0u32, matches[0]);
    for &i in matches {
        let (p, n) = split(i)?;
        if p != product {
            return None;
        }
        if n > best.0 {
            best = (n, i);
        }
    }
    Some(best.1)
}

/// What loading needs from `manifest.yaml`. Parsed with Python's `yaml`
/// (a hard dependency of `huggingface_hub`, so it is present whenever this
/// module runs at all).
struct ManifestBits {
    tcp_default: Option<String>,
    flange_frame: Option<String>,
    mount_frame: Option<String>,
    /// Optical frames a `sensor.camera` package declares (ROS optical
    /// convention: +Z looks, +Y down) — what a wrist camera's axis is
    /// posed from.
    camera_frames: Vec<String>,
    /// Scan-origin frames a `sensor.lidar` package declares (ROS laser
    /// convention: scan plane XY, 0° along +X — botrail's own lidar
    /// frame, so no rotation fix on import).
    lidar_frames: Vec<String>,
    /// Grasp-surface frames a gripper/hand package declares — the
    /// fingertips a grasp is meant to happen between.
    grasp_frames: Vec<String>,
    /// The arms of a dual-arm package (`frames.arms[]`).
    arms: Vec<ManifestArm>,
    /// Maker / product / category / numeric specs — what a bill of
    /// materials names the package by.
    meta: CatalogMeta,
}

/// One entry of `frames.arms[]`.
struct ManifestArm {
    name: String,
    tcp_default: String,
    flange_frame: Option<String>,
    joints: Vec<String>,
}

fn read_manifest(py: Python<'_>, package_dir: &Path) -> PyResult<ManifestBits> {
    let path = package_dir.join("manifest.yaml");
    let text =
        std::fs::read_to_string(&path).map_err(|e| err(format!("{}: {e}", path.display())))?;
    let yaml = py.import("yaml")?;
    let manifest = yaml.call_method1("safe_load", (text,))?;
    let frame = |key: &str| {
        manifest
            .get_item("frames")
            .ok()
            .and_then(|frames| frames.get_item(key).ok())
            .and_then(|v| v.extract::<Option<String>>().ok())
            .flatten()
    };
    let text_at = |keys: &[&str]| -> Option<String> {
        let mut node = manifest.clone();
        for key in keys {
            node = node.get_item(*key).ok()?;
        }
        node.extract::<Option<String>>().ok().flatten()
    };
    // Numeric specs only, in manifest order: `dof`, `payload_kg`,
    // `reach_mm`, `mass_kg`, ... Lists and strings (`controller`,
    // `ip_rating`) are not BOM attributes and are dropped here.
    let mut specs = Vec::new();
    if let Ok(dict) = manifest.get_item("specs") {
        if let Ok(items) = dict.call_method0("items") {
            if let Ok(iter) = items.try_iter() {
                for item in iter.flatten() {
                    let Ok((key, value)) = item.extract::<(String, Bound<'_, PyAny>)>() else {
                        continue;
                    };
                    // Booleans are ints in Python; keep them out of the
                    // numeric column set.
                    if value.is_instance_of::<pyo3::types::PyBool>() {
                        continue;
                    }
                    if let Ok(number) = value.extract::<f64>() {
                        specs.push((key, number));
                    }
                }
            }
        }
    }
    let frame_list = |key: &str| -> Vec<String> {
        manifest
            .get_item("frames")
            .ok()
            .and_then(|frames| frames.get_item(key).ok())
            .and_then(|v| v.extract::<Option<Vec<String>>>().ok())
            .flatten()
            .unwrap_or_default()
    };
    // `frames.arms[]`: an arm without a name or a TCP is not an arm.
    let mut arms = Vec::new();
    if let Ok(list) = manifest
        .get_item("frames")
        .and_then(|frames| frames.get_item("arms"))
    {
        if let Ok(iter) = list.try_iter() {
            for item in iter.flatten() {
                let text = |key: &str| -> Option<String> {
                    item.get_item(key)
                        .ok()
                        .and_then(|v| v.extract::<Option<String>>().ok())
                        .flatten()
                };
                let (Some(name), Some(tcp_default)) = (text("name"), text("tcp_default")) else {
                    continue;
                };
                arms.push(ManifestArm {
                    name,
                    tcp_default,
                    flange_frame: text("flange_frame"),
                    joints: item
                        .get_item("joints")
                        .ok()
                        .and_then(|v| v.extract::<Option<Vec<String>>>().ok())
                        .flatten()
                        .unwrap_or_default(),
                });
            }
        }
    }
    Ok(ManifestBits {
        tcp_default: frame("tcp_default"),
        flange_frame: frame("flange_frame"),
        mount_frame: frame("mount_frame"),
        camera_frames: frame_list("camera_frames"),
        lidar_frames: frame_list("lidar_frames"),
        grasp_frames: frame_list("grasp_frames"),
        arms,
        meta: CatalogMeta {
            manufacturer: text_at(&["manufacturer", "name"]),
            product: text_at(&["name"]),
            category: text_at(&["category"]),
            specs,
        },
    })
}

/// The bits `Scene.add_camera(from_catalog=)` composes: optics defaults
/// from the package's flat specs, the mount→optical offset from its own
/// zero-pose FK — converted from the ROS optical convention (+Z looks,
/// +Y down) to botrail's camera frame (-Z looks, +Y up) — and the
/// identity a BOM line names it by (design-camera.md §11 B4).
pub struct CameraPackage {
    pub fov_h_deg: Option<f64>,
    pub resolution: Option<[u32; 2]>,
    /// `min_range_mm` / `max_range_mm`, meters.
    pub near: Option<f64>,
    pub far: Option<f64>,
    /// Mount-face frame → botrail camera frame; `None` when the package
    /// declares no resolvable optical frame.
    pub optical_offset: Option<nalgebra::Isometry3<f64>>,
    pub id: String,
    pub revision: String,
    pub meta: CatalogMeta,
}

pub fn camera_from_catalog(
    py: Python<'_>,
    query: &str,
    revision: Option<&str>,
) -> PyResult<CameraPackage> {
    let (snapshot, entry, sha) = download_package(py, query, revision)?;
    let bits = read_manifest(py, &snapshot.join(&entry.id))?;
    let spec = |key: &str| {
        bits.meta
            .specs
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| *v)
    };
    let resolution = match (spec("resolution_h_px"), spec("resolution_v_px")) {
        (Some(w), Some(h)) if w >= 1.0 && h >= 1.0 => Some([w as u32, h as u32]),
        _ => None,
    };
    let optical_offset = if bits.camera_frames.is_empty() {
        None
    } else {
        let model = from_catalog(py, query, revision, None)?;
        let q = vec![0.0; model.dof()];
        let poses =
            botrail_kin::forward_kinematics_with_base(&model, &q, &nalgebra::Isometry3::identity())
                .map_err(|e| err(e.to_string()))?;
        // USD link names are prim paths; match by last segment there.
        let find = |name: &str| {
            model.link_index(name).or_else(|| {
                let matches: Vec<usize> = (0..model.links.len())
                    .filter(|&i| model.links[i].name.rsplit('/').next() == Some(name))
                    .collect();
                match matches.as_slice() {
                    [one] => Some(*one),
                    _ => None,
                }
            })
        };
        match bits.camera_frames.iter().find_map(|f| find(f)) {
            Some(optical) => {
                let mount = model
                    .mount_link
                    .map(|m| poses[m])
                    .unwrap_or_else(nalgebra::Isometry3::identity);
                let flip = nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::identity(),
                    nalgebra::UnitQuaternion::from_axis_angle(
                        &nalgebra::Vector3::x_axis(),
                        std::f64::consts::PI,
                    ),
                );
                Some(mount.inverse() * poses[optical] * flip)
            }
            None => {
                eprintln!(
                    "botrail: catalog `{}`: no camera_frames entry matches a link; \
                     the optical axis stays at the mount face",
                    entry.id
                );
                None
            }
        }
    };
    Ok(CameraPackage {
        fov_h_deg: spec("fov_h_deg"),
        resolution,
        near: spec("min_range_mm").map(|v| v / 1000.0),
        far: spec("max_range_mm").map(|v| v / 1000.0),
        optical_offset,
        id: entry.id,
        revision: sha,
        meta: bits.meta,
    })
}

/// The bits `Scene.add_lidar(from_catalog=)` composes: sweep defaults
/// from the package's flat specs, the mount→scan-frame offset from its
/// own zero-pose FK — no convention fix: the catalog's `lidar_frames`
/// are ROS laser frames (scan plane XY, 0° along +X), which is botrail's
/// lidar frame verbatim — and the identity a BOM line names it by
/// (design-lidar.md §11).
pub struct LidarPackage {
    pub scan_fov_deg: Option<f64>,
    pub resolution_deg: Option<f64>,
    /// `min_range_mm` / `max_range_mm`, meters.
    pub range: Option<[f64; 2]>,
    /// Vertical rings of a 3D scanner (`channels` spec, >= 2 to count).
    pub channels: Option<u32>,
    /// Full vertical field, degrees (`vfov_deg` spec).
    pub vfov_deg: Option<f64>,
    /// Mount-face frame → scan frame; `None` when the package declares
    /// no resolvable scan frame.
    pub scan_offset: Option<nalgebra::Isometry3<f64>>,
    pub id: String,
    pub revision: String,
    pub meta: CatalogMeta,
}

pub fn lidar_from_catalog(
    py: Python<'_>,
    query: &str,
    revision: Option<&str>,
) -> PyResult<LidarPackage> {
    let (snapshot, entry, sha) = download_package(py, query, revision)?;
    let bits = read_manifest(py, &snapshot.join(&entry.id))?;
    let spec = |key: &str| {
        bits.meta
            .specs
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| *v)
    };
    let range = match (spec("min_range_mm"), spec("max_range_mm")) {
        (Some(min), Some(max)) if min > 0.0 && max > min => Some([min / 1000.0, max / 1000.0]),
        _ => None,
    };
    let scan_offset = if bits.lidar_frames.is_empty() {
        None
    } else {
        let model = from_catalog(py, query, revision, None)?;
        let q = vec![0.0; model.dof()];
        let poses =
            botrail_kin::forward_kinematics_with_base(&model, &q, &nalgebra::Isometry3::identity())
                .map_err(|e| err(e.to_string()))?;
        // USD link names are prim paths; match by last segment there.
        let find = |name: &str| {
            model.link_index(name).or_else(|| {
                let matches: Vec<usize> = (0..model.links.len())
                    .filter(|&i| model.links[i].name.rsplit('/').next() == Some(name))
                    .collect();
                match matches.as_slice() {
                    [one] => Some(*one),
                    _ => None,
                }
            })
        };
        match bits.lidar_frames.iter().find_map(|f| find(f)) {
            Some(scan) => {
                let mount = model
                    .mount_link
                    .map(|m| poses[m])
                    .unwrap_or_else(nalgebra::Isometry3::identity);
                Some(mount.inverse() * poses[scan])
            }
            None => {
                eprintln!(
                    "botrail: catalog `{}`: no lidar_frames entry matches a link; \
                     the scan origin stays at the mount face",
                    entry.id
                );
                None
            }
        }
    };
    Ok(LidarPackage {
        scan_fov_deg: spec("scan_fov_deg"),
        resolution_deg: spec("angular_resolution_deg"),
        range,
        // A 3D scanner's rings and vertical field travel together; a
        // package declaring one without the other keeps its planar
        // default rather than authoring a degenerate sweep.
        channels: match (spec("channels"), spec("vfov_deg")) {
            (Some(c), Some(_)) if c >= 2.0 => Some(c.round() as u32),
            _ => None,
        },
        vfov_deg: match (spec("channels"), spec("vfov_deg")) {
            (Some(c), Some(v)) if c >= 2.0 => Some(v),
            _ => None,
        },
        scan_offset,
        id: entry.id,
        revision: sha,
        meta: bits.meta,
    })
}

/// The pyo3-facing wrapper: builds the model and wraps it for Python.
pub fn robot_from_catalog(
    py: Python<'_>,
    query: &str,
    revision: Option<&str>,
    format: Option<&str>,
) -> PyResult<Arc<RobotModel>> {
    Ok(Arc::new(from_catalog(py, query, revision, format)?))
}
