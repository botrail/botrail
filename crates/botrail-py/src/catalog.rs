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
//!    `robotiq/2f/2f-85/r1` as long as they are unambiguous. Among revisions
//!    of one product, shorthand prefers the newest publicly distributed one.
//! 3. `snapshot_download(allow_patterns=[<id>/*])` fetches the package;
//!    `manifest.yaml` supplies the model asset paths and
//!    `frames.tcp_default`.

use nalgebra::{Isometry3, Translation3, UnitQuaternion};
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

/// Manifest names may be plain link names or kit alias/link names. USD
/// prim ancestry is retained in the model, and must resolve unambiguously.
fn resolve_frame(model: &RobotModel, name: &str) -> Option<usize> {
    model.link_index(name).or_else(|| {
        let (prefix, leaf) = name.rsplit_once('/').unwrap_or(("", name));
        let found: Vec<_> = model
            .links
            .iter()
            .enumerate()
            .filter(|(_, link)| {
                link.name.rsplit('/').next() == Some(leaf)
                    && (prefix.is_empty() || link.name.starts_with(&format!("{prefix}/")))
            })
            .map(|(i, _)| i)
            .collect();
        if prefix.is_empty() {
            // A bare name in a kit refers to its base, whose USD prim path
            // starts with '/', while attached parts carry alias/ prefixes.
            let base: Vec<_> = found
                .iter()
                .copied()
                .filter(|&i| model.links[i].name.starts_with('/'))
                .collect();
            if let [one] = base.as_slice() {
                return Some(*one);
            }
        }
        if let [one] = found.as_slice() {
            Some(*one)
        } else {
            None
        }
    })
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
             Robot.from_package (see {REPO_URL})",
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
    validate_format(format)?;
    let (snapshot, entry, sha) = download_package(py, query, revision)?;
    let package_dir = snapshot.join(&entry.id);
    load_package(
        py,
        &package_dir,
        &snapshot,
        entry,
        sha,
        format,
        None,
        &mut Vec::new(),
    )
}

fn validate_format(format: Option<&str>) -> PyResult<()> {
    match format {
        None | Some("urdf") | Some("usd") => {}
        Some(other) => {
            return Err(err(format!(
                "unknown format `{other}`; pass \"urdf\", \"usd\", or omit it"
            )))
        }
    }
    Ok(())
}

/// A builder output uses the same manifest/frame/mounting reader as Hub assets.
pub fn from_package(
    py: Python<'_>,
    path: &Path,
    format: Option<&str>,
    catalog_root: Option<&Path>,
) -> PyResult<RobotModel> {
    validate_format(format)?;
    from_local(py, path, format, catalog_root, &mut Vec::new())
}

fn from_local(
    py: Python<'_>,
    path: &Path,
    format: Option<&str>,
    catalog_root: Option<&Path>,
    stack: &mut Vec<String>,
) -> PyResult<RobotModel> {
    let package_dir = path
        .canonicalize()
        .map_err(|e| err(format!("{}: {e}", path.display())))?;
    let (id, urdf, usd, revision): (String, Option<String>, Option<String>, String) = py
        .import("botrail.catalog")?
        .call_method1("_package_info", (&package_dir,))?
        .extract()?;
    let inferred = package_dir
        .ends_with(&id)
        .then(|| package_dir.ancestors().nth(4).unwrap().to_path_buf());
    let root = catalog_root.or(inferred.as_deref());
    let entry = IndexEntry {
        id,
        distribution: String::new(),
        urdf,
        usd,
    };
    load_package(
        py,
        &package_dir,
        &package_dir,
        entry,
        revision,
        format,
        root,
        stack,
    )
}

#[allow(clippy::too_many_arguments)]
fn load_package(
    py: Python<'_>,
    package_dir: &Path,
    asset_root: &Path,
    entry: IndexEntry,
    mut sha: String,
    format: Option<&str>,
    local_root: Option<&Path>,
    stack: &mut Vec<String>,
) -> PyResult<RobotModel> {
    if stack.contains(&entry.id) || stack.len() >= 32 {
        return Err(err(format!(
            "cyclic or too deeply nested kit dependency: {:?} -> {}",
            stack, entry.id
        )));
    }
    stack.push(entry.id.clone());
    let mut manifest = read_manifest(py, package_dir)?;
    if manifest.id != entry.id {
        return Err(err(format!(
            "package identity mismatch: expected {}, found {}",
            entry.id, manifest.id
        )));
    }
    let mut model = if let Some(kit) = &manifest.meta.kit {
        let mut revisions = Vec::new();
        let mut component = |id: &str| -> PyResult<RobotModel> {
            let model = if sha.starts_with("local-sha256:") {
                let root = local_root.ok_or_else(|| err("local kits need catalog_root pointing to the directory containing component catalog IDs".into()))?;
                let path = root.join(id);
                let resolved = path
                    .canonicalize()
                    .map_err(|e| err(format!("kit component {id}: {e}")))?;
                let root = root.canonicalize().map_err(|e| err(e.to_string()))?;
                if !resolved.starts_with(&root) {
                    return Err(err(format!("kit component {id} escapes catalog_root")));
                }
                from_local(py, &resolved, format, Some(&root), stack)?
            } else {
                let (snapshot, entry, revision) = download_package(py, id, Some(&sha))?;
                load_package(
                    py,
                    &snapshot.join(&entry.id),
                    &snapshot,
                    entry,
                    revision,
                    format,
                    None,
                    stack,
                )?
            };
            match &model.source {
                RobotSource::Catalog {
                    id: actual,
                    revision,
                    ..
                } if actual == id => revisions.push(revision.clone()),
                _ => {
                    return Err(err(format!(
                        "kit component identity mismatch: expected {id}"
                    )))
                }
            }
            Ok(model)
        };
        let mut model = component(&kit.base)?;
        for a in &kit.attachments {
            let tool = component(&a.package)?;
            let [r, p, y] = a.offset.rpy;
            let offset = Isometry3::from_parts(
                Translation3::from(a.offset.xyz),
                UnitQuaternion::from_euler_angles(r, p, y),
            );
            let parent = resolve_frame(&model, &a.parent_frame).ok_or_else(|| {
                err(format!(
                    "kit parent frame {} is missing or ambiguous",
                    a.parent_frame
                ))
            })?;
            let mount = resolve_frame(&tool, &a.mount_frame).ok_or_else(|| {
                err(format!(
                    "kit mount frame {} is missing or ambiguous",
                    a.mount_frame
                ))
            })?;
            model = model
                .attach_tool(
                    &tool,
                    Some(&model.links[parent].name),
                    Some(&tool.links[mount].name),
                    offset,
                    None,
                    Some(&format!("{}/", a.alias)),
                    None,
                )
                .map_err(|e| err(format!("kit {} attachment {}: {e}", entry.id, a.alias)))?;
        }
        if sha.starts_with("local-sha256:") {
            sha = py
                .import("botrail.catalog")?
                .call_method1("_kit_revision", (&sha, revisions))?
                .extract()?;
        }
        model
    } else {
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
        // Hub index paths are repo-relative; local manifest paths package-relative.
        let model_path = asset_root.join(&rel);

        let model = if is_usd {
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

        model
    };

    // The manifest's declared frames beat the heuristics: tcp_default
    // replaces the deepest-leaf guess, flange/mount give `attach_tool` its
    // argument-free defaults. USD link names are prim paths; match by last
    // path segment there.
    let resolve = |field: &str, name: Option<&str>| -> Option<usize> {
        let name = name?;
        let index = resolve_frame(&model, name);
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
    if manifest.meta.kit.is_some() && (tcp.is_none() || mount.is_none()) {
        return Err(err(
            "kit must declare resolvable frames.mount_frame and frames.tcp_default".into(),
        ));
    }
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
    // Use the same exact-or-unique-leaf resolution as the package's frames.
    // Store canonical model names so USD declarations survive source replay.
    if let Some(spec) = &mut manifest.meta.mounting {
        let mut names = std::collections::BTreeMap::new();
        for face in &mut spec.interfaces {
            let index =
                resolve("mounting.interfaces[].frame", Some(&face.frame)).ok_or_else(|| {
                    err(format!(
                        "catalog `{}`: mounting frame `{}` does not exist or is ambiguous",
                        entry.id, face.frame
                    ))
                })?;
            let canonical = model.links[index].name.clone();
            names.insert(face.frame.clone(), canonical.clone());
            face.frame = canonical;
        }
        for req in &mut spec.requirements {
            req.frame = names[&req.frame].clone();
        }
        spec.validate(&manifest.meta.sources, manifest.meta.order.as_ref())
            .map_err(err)?;
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

    if manifest.meta.kit.is_some() {
        model.name = entry.id.split('/').nth(2).unwrap_or(&entry.id).to_string();
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
    stack.pop();
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
    // Resolve product identity before preferring public distribution: a
    // metadata-only revision must not displace a usable release, and the
    // availability of a different product must not resolve an ambiguity.
    // Exact IDs (including kit dependencies) bypass this preference above.
    // The selected full ID and dataset SHA are recorded for replay.
    if matches.len() > 1 {
        if let Some(preferred) = preferred_revision(&entries, &matches) {
            matches = vec![preferred];
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

/// Prefer the newest public revision of one product. If none is public,
/// return the newest entry so distribution gating explains the local build.
/// Different products or unorderable revisions remain ambiguous.
fn preferred_revision(entries: &[IndexEntry], matches: &[usize]) -> Option<usize> {
    let split = |i: usize| -> Option<(&str, u32)> {
        let id = entries[i].id.as_str();
        let (product, rev) = id.rsplit_once('/')?;
        let n = rev.strip_prefix('r')?.parse::<u32>().ok()?;
        Some((product, n))
    };
    let (product, _) = split(matches[0])?;
    let mut best = ((false, 0u32), matches[0]);
    for &i in matches {
        let (p, n) = split(i)?;
        if p != product {
            return None;
        }
        let priority = (entries[i].distribution == "public", n);
        if priority > best.0 {
            best = (priority, i);
        }
    }
    Some(best.1)
}

/// What loading needs from `manifest.yaml`. Parsed with Python's `yaml`
/// (a hard dependency of `huggingface_hub`, so it is present whenever this
/// module runs at all).
struct ManifestBits {
    id: String,
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
    // YAML dates in source provenance are serialized as ISO text. Unlike the
    // legacy optional frame hints, malformed mounting declarations are errors.
    let field = |key: &str| -> PyResult<String> {
        let value = manifest.call_method1("get", (key,))?;
        let kwargs = PyDict::new(py);
        kwargs.set_item("default", py.import("builtins")?.getattr("str")?)?;
        py.import("json")?
            .call_method("dumps", (value,), Some(&kwargs))?
            .extract()
    };
    let mounting: Option<botrail_model::mounting::MountingSpec> =
        serde_json::from_str(&field("mounting")?)
            .map_err(|e| err(format!("{}: mounting: {e}", path.display())))?;
    let order: Option<botrail_model::mounting::CatalogOrder> =
        serde_json::from_str(&field("order")?)
            .map_err(|e| err(format!("{}: order: {e}", path.display())))?;
    let sources: Option<Vec<botrail_model::mounting::CatalogSource>> =
        serde_json::from_str(&field("sources")?)
            .map_err(|e| err(format!("{}: sources: {e}", path.display())))?;
    let sources = sources.unwrap_or_default();
    let kit: Option<botrail_model::kit::KitSpec> = serde_json::from_str(&field("kit")?)
        .map_err(|e| err(format!("{}: kit: {e}", path.display())))?;
    let compatibility: Option<botrail_model::compatibility::Compatibility> =
        serde_json::from_str(&field("compatibility")?)
            .map_err(|e| err(format!("{}: compatibility: {e}", path.display())))?;
    let compatibility = compatibility.unwrap_or_default();
    compatibility.validate(&sources).map_err(err)?;
    let electrical =
        serde_json::from_str(&field("electrical")?).map_err(|e| err(format!("electrical: {e}")))?;
    let kind: Option<String> =
        serde_json::from_str(&field("kind")?).map_err(|e| err(e.to_string()))?;
    if (kind.as_deref() == Some("kit")) != kit.is_some() {
        return Err(err(
            "kind: kit and kit content must be declared together".into()
        ));
    }
    if kit.is_some() && mounting.is_some() {
        return Err(err(
            "kit mounting declarations belong to its components".into()
        ));
    }
    if let Some(kit) = &kit {
        kit.validate(&sources, order.as_ref()).map_err(err)?;
    }

    if let Some(spec) = &mounting {
        spec.validate(&sources, order.as_ref())
            .map_err(|e| err(format!("{}: {e}", path.display())))?;
    }
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
        id: text_at(&["id"]).ok_or_else(|| err("manifest requires id".into()))?,
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
            mounting,
            kit,
            compatibility,
            electrical,
            order,
            sources,
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
