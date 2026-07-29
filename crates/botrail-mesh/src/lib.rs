//! Minimal triangle-mesh loading for collision shapes: binary/ASCII STL and
//! a practical OBJ subset (v/f statements, n-gon fan triangulation,
//! negative indices). Parsers work on bytes, so callers can feed files,
//! archives, or in-memory assets (wasm) alike.
//!
//! Deliberately not a general asset pipeline: no normals, UVs, or
//! materials — collision geometry only needs positions and triangles.
//! DAE/glTF and anything scene-graph-shaped belong to the importer layer.

use std::path::Path;

use thiserror::Error;

#[derive(Debug, Error)]
pub enum MeshError {
    #[error("failed to read `{path}`: {source}")]
    Io {
        path: String,
        source: std::io::Error,
    },
    #[error("unsupported mesh format `{0}` (supported: stl, obj)")]
    UnsupportedFormat(String),
    #[error("{0}")]
    Parse(String),
    #[error("mesh has no triangles")]
    Empty,
}

/// Indexed triangle mesh, positions only.
#[derive(Debug, Clone, PartialEq)]
pub struct MeshData {
    pub vertices: Vec<[f64; 3]>,
    pub indices: Vec<[u32; 3]>,
}

impl MeshData {
    /// Per-axis scaled copy (URDF `<mesh scale="...">` semantics).
    pub fn scaled(&self, scale: [f64; 3]) -> MeshData {
        MeshData {
            vertices: self
                .vertices
                .iter()
                .map(|v| [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]])
                .collect(),
            indices: self.indices.clone(),
        }
    }
}

/// Loads a mesh file, picking the parser from the (lowercased) extension.
pub fn load_path(path: &Path) -> Result<MeshData, MeshError> {
    let ext = path
        .extension()
        .map(|e| e.to_string_lossy().to_lowercase())
        .unwrap_or_default();
    let bytes = std::fs::read(path).map_err(|source| MeshError::Io {
        path: path.display().to_string(),
        source,
    })?;
    match ext.as_str() {
        "stl" => parse_stl(&bytes),
        "obj" => parse_obj(&String::from_utf8_lossy(&bytes)),
        other => Err(MeshError::UnsupportedFormat(other.to_string())),
    }
}

// ------------------------------------------------------------------- STL

/// Parses an STL file, auto-detecting binary vs ASCII. Binary detection
/// goes by the size formula rather than the `solid` prefix, which binary
/// exporters also emit.
pub fn parse_stl(bytes: &[u8]) -> Result<MeshData, MeshError> {
    if bytes.len() >= 84 {
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        if bytes.len() == 84 + count * 50 {
            return parse_stl_binary(bytes, count);
        }
    }
    parse_stl_ascii(&String::from_utf8_lossy(bytes))
}

fn parse_stl_binary(bytes: &[u8], count: usize) -> Result<MeshData, MeshError> {
    let mut vertices = Vec::with_capacity(count * 3);
    let mut indices = Vec::with_capacity(count);
    for t in 0..count {
        // 50-byte record: normal (3xf32), 3 vertices (9xf32), u16 attribute.
        let record = &bytes[84 + t * 50..84 + (t + 1) * 50];
        for v in 0..3 {
            let at = 12 + v * 12;
            let read = |o: usize| {
                f32::from_le_bytes(record[at + o..at + o + 4].try_into().unwrap()) as f64
            };
            vertices.push([read(0), read(4), read(8)]);
        }
        let base = (t * 3) as u32;
        indices.push([base, base + 1, base + 2]);
    }
    if indices.is_empty() {
        return Err(MeshError::Empty);
    }
    Ok(MeshData { vertices, indices })
}

fn parse_stl_ascii(text: &str) -> Result<MeshData, MeshError> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut facet: Vec<[f64; 3]> = Vec::with_capacity(3);
    for (line_no, line) in text.lines().enumerate() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("vertex") => {
                let mut read = || -> Result<f64, MeshError> {
                    words
                        .next()
                        .and_then(|w| w.parse().ok())
                        .ok_or_else(|| MeshError::Parse(format!("stl line {}: bad vertex", line_no + 1)))
                };
                facet.push([read()?, read()?, read()?]);
            }
            Some("endfacet") => {
                if facet.len() != 3 {
                    return Err(MeshError::Parse(format!(
                        "stl line {}: facet with {} vertices",
                        line_no + 1,
                        facet.len()
                    )));
                }
                let base = vertices.len() as u32;
                vertices.extend(facet.drain(..));
                indices.push([base, base + 1, base + 2]);
            }
            _ => {}
        }
    }
    if indices.is_empty() {
        return Err(MeshError::Empty);
    }
    Ok(MeshData { vertices, indices })
}

// ------------------------------------------------------------------- OBJ

/// Parses the OBJ subset that matters for collision geometry: `v` and `f`
/// statements. Faces may be n-gons (fan-triangulated) and use any of the
/// `i`, `i/t`, `i//n`, `i/t/n` index forms, including negative (relative)
/// indices.
pub fn parse_obj(text: &str) -> Result<MeshData, MeshError> {
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut indices = Vec::new();
    for (line_no, line) in text.lines().enumerate() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("v") => {
                let mut read = || -> Result<f64, MeshError> {
                    words
                        .next()
                        .and_then(|w| w.parse().ok())
                        .ok_or_else(|| MeshError::Parse(format!("obj line {}: bad vertex", line_no + 1)))
                };
                vertices.push([read()?, read()?, read()?]);
            }
            Some("f") => {
                let mut face = Vec::with_capacity(4);
                for word in words {
                    let index_str = word.split('/').next().unwrap_or("");
                    let raw: i64 = index_str.parse().map_err(|_| {
                        MeshError::Parse(format!("obj line {}: bad face index `{word}`", line_no + 1))
                    })?;
                    let index = if raw < 0 {
                        vertices.len() as i64 + raw
                    } else {
                        raw - 1
                    };
                    if index < 0 || index as usize >= vertices.len() {
                        return Err(MeshError::Parse(format!(
                            "obj line {}: face index {raw} out of range",
                            line_no + 1
                        )));
                    }
                    face.push(index as u32);
                }
                if face.len() < 3 {
                    return Err(MeshError::Parse(format!(
                        "obj line {}: face with {} vertices",
                        line_no + 1,
                        face.len()
                    )));
                }
                for k in 1..face.len() - 1 {
                    indices.push([face[0], face[k], face[k + 1]]);
                }
            }
            _ => {}
        }
    }
    if indices.is_empty() {
        return Err(MeshError::Empty);
    }
    Ok(MeshData { vertices, indices })
}

// ------------------------------------------------------------- test aids

/// An axis-aligned box mesh (12 triangles), centered at the origin — handy
/// for tests and as a fallback shape.
pub fn box_mesh(size: [f64; 3]) -> MeshData {
    let [hx, hy, hz] = [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0];
    let vertices = vec![
        [-hx, -hy, -hz],
        [hx, -hy, -hz],
        [hx, hy, -hz],
        [-hx, hy, -hz],
        [-hx, -hy, hz],
        [hx, -hy, hz],
        [hx, hy, hz],
        [-hx, hy, hz],
    ];
    let indices = vec![
        [0, 2, 1],
        [0, 3, 2], // bottom
        [4, 5, 6],
        [4, 6, 7], // top
        [0, 1, 5],
        [0, 5, 4], // -y
        [2, 3, 7],
        [2, 7, 6], // +y
        [1, 2, 6],
        [1, 6, 5], // +x
        [3, 0, 4],
        [3, 4, 7], // -x
    ];
    MeshData { vertices, indices }
}

/// Serializes a mesh as binary STL (little-endian), for tests and asset
/// generation.
pub fn to_stl_binary(mesh: &MeshData) -> Vec<u8> {
    let mut out = vec![0u8; 80];
    out.extend_from_slice(&(mesh.indices.len() as u32).to_le_bytes());
    for tri in &mesh.indices {
        out.extend_from_slice(&[0u8; 12]); // normal (unused)
        for &i in tri {
            for c in mesh.vertices[i as usize] {
                out.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&[0u8; 2]); // attribute bytes
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_stl_roundtrip() {
        let mesh = box_mesh([0.2, 0.4, 0.6]);
        let bytes = to_stl_binary(&mesh);
        let parsed = parse_stl(&bytes).unwrap();
        assert_eq!(parsed.indices.len(), 12);
        assert_eq!(parsed.vertices.len(), 36); // deduplication is not required
        let max_z = parsed.vertices.iter().map(|v| v[2]).fold(f64::MIN, f64::max);
        assert!((max_z - 0.3).abs() < 1e-6);
    }

    #[test]
    fn ascii_stl_parses() {
        let text = "solid t\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0\n  endloop\n endfacet\nendsolid t\n";
        let mesh = parse_stl(text.as_bytes()).unwrap();
        assert_eq!(mesh.indices.len(), 1);
        assert_eq!(mesh.vertices[1], [1.0, 0.0, 0.0]);
    }

    #[test]
    fn obj_quads_and_negative_indices() {
        let text = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\nf -4 -3 -2\n";
        let mesh = parse_obj(text).unwrap();
        // Quad fans into 2 triangles + 1 explicit = 3.
        assert_eq!(mesh.indices.len(), 3);
        assert_eq!(mesh.indices[0], [0, 1, 2]);
        assert_eq!(mesh.indices[1], [0, 2, 3]);
        assert_eq!(mesh.indices[2], [0, 1, 2]);
    }

    #[test]
    fn obj_slash_forms() {
        let text = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/1 2//2 3/3/3\n";
        let mesh = parse_obj(text).unwrap();
        assert_eq!(mesh.indices, vec![[0, 1, 2]]);
    }

    #[test]
    fn scaled_scales_vertices_only() {
        let mesh = box_mesh([2.0, 2.0, 2.0]).scaled([1.0, 2.0, 3.0]);
        let max_y = mesh.vertices.iter().map(|v| v[1]).fold(f64::MIN, f64::max);
        let max_z = mesh.vertices.iter().map(|v| v[2]).fold(f64::MIN, f64::max);
        assert!((max_y - 2.0).abs() < 1e-12);
        assert!((max_z - 3.0).abs() < 1e-12);
    }

    #[test]
    fn empty_and_garbage_are_errors() {
        assert!(matches!(parse_obj("v 0 0 0\n"), Err(MeshError::Empty)));
        assert!(parse_stl(b"not a mesh at all").is_err());
        assert!(matches!(
            parse_obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n"),
            Err(MeshError::Parse(_))
        ));
    }
}
