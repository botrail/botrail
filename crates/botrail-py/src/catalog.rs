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

use botrail_model::{RobotModel, RobotSource};
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
    let hub = py.import("huggingface_hub").map_err(|_| {
        err(
            "Robot.from_catalog needs the optional dependency `huggingface_hub` — install it \
             with `pip install botrail[catalog]`"
                .to_string(),
        )
    })?;

    // Pin the revision to a commit SHA before any download, so every file
    // comes from one consistent snapshot and projects can replay it.
    let kwargs = PyDict::new(py);
    kwargs.set_item("repo_type", "dataset")?;
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
    let package_dir = Path::new(&snapshot).join(&entry.id);

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
    let model_path = Path::new(&snapshot).join(&rel);

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

    // The manifest's declared TCP beats the deepest-leaf heuristic. USD
    // link names are prim paths; match by last path segment there.
    let tcp = manifest.tcp_default.as_deref().and_then(|name| {
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
                "botrail: catalog `{}`: manifest tcp_default `{name}` is not a link; using the \
                 default TCP heuristic",
                entry.id
            );
        }
        index
    });
    model.tcp_link = tcp;

    let inner = std::mem::replace(&mut model.source, RobotSource::UrdfXml(String::new()));
    model.source = RobotSource::Catalog {
        id: entry.id,
        revision: sha,
        tcp: tcp.map(|i| model.links[i].name.clone()),
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
    let matches: Vec<usize> = entries
        .iter()
        .enumerate()
        .filter(|(_, e)| is_subsequence(&e.id))
        .map(|(i, _)| i)
        .collect();
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

/// What loading needs from `manifest.yaml`. Parsed with Python's `yaml`
/// (a hard dependency of `huggingface_hub`, so it is present whenever this
/// module runs at all).
struct ManifestBits {
    tcp_default: Option<String>,
}

fn read_manifest(py: Python<'_>, package_dir: &Path) -> PyResult<ManifestBits> {
    let path = package_dir.join("manifest.yaml");
    let text =
        std::fs::read_to_string(&path).map_err(|e| err(format!("{}: {e}", path.display())))?;
    let yaml = py.import("yaml")?;
    let manifest = yaml.call_method1("safe_load", (text,))?;
    let tcp_default = manifest
        .get_item("frames")
        .ok()
        .and_then(|frames| frames.get_item("tcp_default").ok())
        .and_then(|v| v.extract::<Option<String>>().ok())
        .flatten();
    Ok(ManifestBits { tcp_default })
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
