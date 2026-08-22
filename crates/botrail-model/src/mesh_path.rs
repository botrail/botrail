//! Resolution of URDF mesh URIs (`package://`, `file://`, relative paths)
//! without a ROS installation.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Default)]
pub struct ModelOptions {
    /// Maps ROS package names to directories for resolving `package://` URIs.
    pub package_paths: HashMap<String, PathBuf>,
    /// Fills a xacro's `$(arg …)` substitutions, so one parametric file can
    /// be expanded to the size at hand. Ignored by the plain URDF readers.
    pub xacro_args: HashMap<String, String>,
}

/// Resolves a URDF mesh URI to a filesystem path. Resolution is best-effort:
/// if nothing matches, the raw path is returned so the caller can surface a
/// useful "file not found" error later.
pub fn resolve(filename: &str, base_dir: Option<&Path>, options: &ModelOptions) -> PathBuf {
    if let Some(rest) = filename.strip_prefix("file://") {
        return PathBuf::from(rest);
    }
    if let Some(rest) = filename.strip_prefix("package://") {
        return resolve_package(rest, base_dir, options);
    }
    let path = Path::new(filename);
    if path.is_absolute() {
        return path.to_path_buf();
    }
    match base_dir {
        Some(dir) => dir.join(path),
        None => path.to_path_buf(),
    }
}

fn resolve_package(rest: &str, base_dir: Option<&Path>, options: &ModelOptions) -> PathBuf {
    let (package, rel) = match rest.split_once('/') {
        Some((p, r)) => (p, r),
        None => (rest, ""),
    };
    if let Some(dir) = options.package_paths.get(package) {
        return dir.join(rel);
    }
    if let Some(base) = base_dir {
        // Typical layout: the URDF lives inside the package
        // (e.g. <pkg>/urdf/robot.urdf, meshes at <pkg>/meshes/...).
        for ancestor in base.ancestors() {
            if ancestor.file_name().is_some_and(|n| n == package) {
                return ancestor.join(rel);
            }
        }
        // Sibling package: <workspace>/<pkg>/...
        for ancestor in base.ancestors() {
            let candidate = ancestor.join(package).join(rel);
            if candidate.exists() {
                return candidate;
            }
        }
    }
    PathBuf::from(format!("package://{rest}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolves_file_scheme_and_relative() {
        let opts = ModelOptions::default();
        assert_eq!(
            resolve("file:///abs/mesh.stl", None, &opts),
            PathBuf::from("/abs/mesh.stl")
        );
        assert_eq!(
            resolve("meshes/a.stl", Some(Path::new("/robot/urdf")), &opts),
            PathBuf::from("/robot/urdf/meshes/a.stl")
        );
    }

    #[test]
    fn resolves_package_from_explicit_mapping() {
        let mut opts = ModelOptions::default();
        opts.package_paths
            .insert("my_robot".to_string(), PathBuf::from("/opt/my_robot"));
        assert_eq!(
            resolve("package://my_robot/meshes/a.stl", None, &opts),
            PathBuf::from("/opt/my_robot/meshes/a.stl")
        );
    }

    #[test]
    fn resolves_package_from_urdf_location() {
        // URDF at <...>/my_robot/urdf/robot.urdf, mesh at package://my_robot/meshes/a.stl
        let opts = ModelOptions::default();
        assert_eq!(
            resolve(
                "package://my_robot/meshes/a.stl",
                Some(Path::new("/ws/src/my_robot/urdf")),
                &opts
            ),
            PathBuf::from("/ws/src/my_robot/meshes/a.stl")
        );
    }

    #[test]
    fn unresolvable_package_is_kept_verbatim() {
        let opts = ModelOptions::default();
        assert_eq!(
            resolve("package://nope/meshes/a.stl", None, &opts),
            PathBuf::from("package://nope/meshes/a.stl")
        );
    }
}
