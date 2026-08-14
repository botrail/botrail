//! Minimal triangle-mesh loading for collision shapes: binary/ASCII STL and
//! a practical OBJ subset (v/f statements, n-gon fan triangulation,
//! negative indices). Parsers work on bytes, so callers can feed files,
//! archives, or in-memory assets (wasm) alike.
//!
//! Deliberately not a general asset pipeline: no normals or UVs — collision
//! geometry only needs positions and triangles. The one exception is
//! *diffuse color*: an OBJ that names an `mtllib` carries the colors its
//! manufacturer authored, and dropping them means every downstream picture
//! has to invent a palette instead. [`load_path`] resolves it (it is the
//! only entry point that knows where the file sits); the in-memory parsers
//! leave [`MeshData::face_colors`] empty.
//!
//! DAE/glTF and anything scene-graph-shaped belong to the importer layer.

use std::collections::HashMap;
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

/// Indexed triangle mesh: positions, triangles, and — when the source
/// assigned materials — one diffuse color per triangle.
#[derive(Debug, Clone, PartialEq)]
pub struct MeshData {
    pub vertices: Vec<[f64; 3]>,
    pub indices: Vec<[u32; 3]>,
    /// Diffuse color per triangle, aligned with `indices`. Empty when the
    /// source carried no materials. Linear RGB, which is what USD
    /// `primvars:displayColor` (and three's working space) expect — OBJ
    /// `Kd` is already linear.
    pub face_colors: Vec<[f32; 3]>,
}

impl MeshData {
    /// Positions and triangles, no material colors.
    pub fn new(vertices: Vec<[f64; 3]>, indices: Vec<[u32; 3]>) -> MeshData {
        MeshData {
            vertices,
            indices,
            face_colors: Vec::new(),
        }
    }

    /// Per-axis scaled copy (URDF `<mesh scale="...">` semantics).
    pub fn scaled(&self, scale: [f64; 3]) -> MeshData {
        MeshData {
            vertices: self
                .vertices
                .iter()
                .map(|v| [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]])
                .collect(),
            indices: self.indices.clone(),
            face_colors: self.face_colors.clone(),
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
        "obj" => {
            let text = String::from_utf8_lossy(&bytes);
            parse_obj_with_materials(&text, &mtl_palette(path, &text))
        }
        other => Err(MeshError::UnsupportedFormat(other.to_string())),
    }
}

/// The `mtllib` beside an OBJ, parsed. Empty when the file names none or
/// the library cannot be read — a mesh whose material file did not travel
/// with it still loads, just without colors.
fn mtl_palette(obj_path: &Path, text: &str) -> HashMap<String, [f32; 3]> {
    let Some(dir) = obj_path.parent() else {
        return HashMap::new();
    };
    let mut palette = HashMap::new();
    for line in text.lines() {
        let mut words = line.split_whitespace();
        if words.next() != Some("mtllib") {
            continue;
        }
        // `mtllib` may name several libraries; later ones win on collision,
        // as they do in the OBJ spec's read order.
        for name in words {
            if let Ok(mtl) = std::fs::read(dir.join(name)) {
                palette.extend(parse_mtl(&String::from_utf8_lossy(&mtl)));
            }
        }
    }
    palette
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
    Ok(MeshData::new(vertices, indices))
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
                    words.next().and_then(|w| w.parse().ok()).ok_or_else(|| {
                        MeshError::Parse(format!("stl line {}: bad vertex", line_no + 1))
                    })
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
                vertices.append(&mut facet);
                indices.push([base, base + 1, base + 2]);
            }
            _ => {}
        }
    }
    if indices.is_empty() {
        return Err(MeshError::Empty);
    }
    Ok(MeshData::new(vertices, indices))
}

// ------------------------------------------------------------------- OBJ

/// Parses the OBJ subset that matters for collision geometry: `v` and `f`
/// statements. Faces may be n-gons (fan-triangulated) and use any of the
/// `i`, `i/t`, `i//n`, `i/t/n` index forms, including negative (relative)
/// indices. Material assignments are ignored; use
/// [`parse_obj_with_materials`] to keep them.
pub fn parse_obj(text: &str) -> Result<MeshData, MeshError> {
    parse_obj_with_materials(text, &Default::default())
}

/// [`parse_obj`], keeping `usemtl` assignments: every triangle gets the
/// diffuse color of the material in force when it was declared, looked up
/// in `palette` (see [`parse_mtl`]). Faces declared under no material — or
/// under one the palette does not name — get no color, and a mesh where
/// *nothing* resolves comes back with `face_colors` empty rather than a
/// run of defaults nobody authored.
pub fn parse_obj_with_materials(
    text: &str,
    palette: &HashMap<String, [f32; 3]>,
) -> Result<MeshData, MeshError> {
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();
    let mut face_colors: Vec<Option<[f32; 3]>> = Vec::new();
    let mut current: Option<[f32; 3]> = None;
    for (line_no, line) in text.lines().enumerate() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("usemtl") => {
                current = words.next().and_then(|name| palette.get(name)).copied();
            }
            Some("v") => {
                let mut read = || -> Result<f64, MeshError> {
                    words.next().and_then(|w| w.parse().ok()).ok_or_else(|| {
                        MeshError::Parse(format!("obj line {}: bad vertex", line_no + 1))
                    })
                };
                vertices.push([read()?, read()?, read()?]);
            }
            Some("f") => {
                let mut face = Vec::with_capacity(4);
                for word in words {
                    let index_str = word.split('/').next().unwrap_or("");
                    let raw: i64 = index_str.parse().map_err(|_| {
                        MeshError::Parse(format!(
                            "obj line {}: bad face index `{word}`",
                            line_no + 1
                        ))
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
                    face_colors.push(current);
                }
            }
            _ => {}
        }
    }
    if indices.is_empty() {
        return Err(MeshError::Empty);
    }
    let mut mesh = MeshData::new(vertices, indices);
    if face_colors.iter().any(Option::is_some) {
        // A gap in the middle would misalign every later face, so unmatched
        // triangles take the neutral rather than being dropped.
        mesh.face_colors = face_colors
            .into_iter()
            .map(|c| c.unwrap_or(NEUTRAL_DIFFUSE))
            .collect();
    }
    Ok(mesh)
}

/// What a triangle with no resolvable material is colored.
const NEUTRAL_DIFFUSE: [f32; 3] = [0.6, 0.6, 0.6];

/// Diffuse colors (`Kd`) by material name, from a Wavefront `.mtl`. Values
/// are taken as linear, which is what the OBJ/MTL pair written from
/// COLLADA or glTF sources carries.
pub fn parse_mtl(text: &str) -> HashMap<String, [f32; 3]> {
    let mut out = HashMap::new();
    let mut name: Option<String> = None;
    for line in text.lines() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("newmtl") => name = words.next().map(str::to_string),
            Some("Kd") => {
                let rgb: Vec<f32> = words.filter_map(|w| w.parse().ok()).collect();
                if let (Some(n), [r, g, b]) = (name.as_ref(), rgb.as_slice()) {
                    out.insert(n.clone(), [*r, *g, *b]);
                }
            }
            _ => {}
        }
    }
    out
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
    MeshData::new(vertices, indices)
}

/// Serializes a mesh as Wavefront OBJ plus a companion MTL, carrying the
/// per-face colors as material runs (`usemtl`) — the one format both the
/// studio loader and the USD exporter read face colors back from (STL
/// carries none). `mtl_name` is the file name written into `mtllib`;
/// faces without a color fall back to a neutral grey material.
pub fn to_obj_with_mtl(mesh: &MeshData, mtl_name: &str) -> (String, String) {
    let color_of = |face: usize| -> [f32; 3] {
        mesh.face_colors
            .get(face)
            .copied()
            .unwrap_or([0.7, 0.7, 0.7])
    };
    // Distinct colors -> materials, in first-appearance order.
    let mut colors: Vec<[f32; 3]> = Vec::new();
    let mut material_of = Vec::with_capacity(mesh.indices.len());
    for face in 0..mesh.indices.len() {
        let color = color_of(face);
        let index = match colors.iter().position(|c| *c == color) {
            Some(i) => i,
            None => {
                colors.push(color);
                colors.len() - 1
            }
        };
        material_of.push(index);
    }

    let mut mtl = String::new();
    for (i, c) in colors.iter().enumerate() {
        mtl.push_str(&format!("newmtl m{i}\nKd {} {} {}\n", c[0], c[1], c[2]));
    }

    let mut obj = format!("mtllib {mtl_name}\n");
    for v in &mesh.vertices {
        obj.push_str(&format!("v {} {} {}\n", v[0], v[1], v[2]));
    }
    let mut current: Option<usize> = None;
    for (face, tri) in mesh.indices.iter().enumerate() {
        let material = material_of[face];
        if current != Some(material) {
            obj.push_str(&format!("usemtl m{material}\n"));
            current = Some(material);
        }
        obj.push_str(&format!("f {} {} {}\n", tri[0] + 1, tri[1] + 1, tri[2] + 1));
    }
    (obj, mtl)
}

/// Serializes a mesh as binary STL (little-endian), for tests and asset
/// generation.
pub fn to_stl_binary(mesh: &MeshData) -> Vec<u8> {
    let mut out = vec![0u8; 80];
    out.extend_from_slice(&(mesh.indices.len() as u32).to_le_bytes());
    for tri in &mesh.indices {
        // Face normal — viewers shade flat black on all-zero normals.
        let [a, b, c] = [
            mesh.vertices[tri[0] as usize],
            mesh.vertices[tri[1] as usize],
            mesh.vertices[tri[2] as usize],
        ];
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        for k in n {
            let value = if len > 1e-12 { k / len } else { 0.0 };
            out.extend_from_slice(&(value as f32).to_le_bytes());
        }
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
mod material_tests {
    use super::*;

    const CUBE_OBJ: &str = "\
mtllib parts.mtl
v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
usemtl yellow
f 1 2 3
usemtl grey
f 1 3 4
";

    fn palette() -> HashMap<String, [f32; 3]> {
        parse_mtl("newmtl yellow\nKd 1.0 1.0 0.0\nKs 0 0 0\nnewmtl grey\nKd 0.5 0.5 0.5\n")
    }

    #[test]
    fn mtl_gives_a_diffuse_per_material() {
        let p = palette();
        assert_eq!(p["yellow"], [1.0, 1.0, 0.0]);
        assert_eq!(p["grey"], [0.5, 0.5, 0.5]);
        assert_eq!(p.len(), 2);
    }

    #[test]
    fn usemtl_colors_every_triangle_it_covers() {
        let mesh = parse_obj_with_materials(CUBE_OBJ, &palette()).unwrap();
        assert_eq!(mesh.indices.len(), 2);
        assert_eq!(mesh.face_colors, vec![[1.0, 1.0, 0.0], [0.5, 0.5, 0.5]]);
    }

    #[test]
    fn an_ngon_carries_its_material_across_the_fan() {
        // One quad under one material becomes two triangles, both colored.
        let obj = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nusemtl yellow\nf 1 2 3 4\n";
        let mesh = parse_obj_with_materials(obj, &palette()).unwrap();
        assert_eq!(mesh.indices.len(), 2);
        assert_eq!(mesh.face_colors, vec![[1.0, 1.0, 0.0]; 2]);
    }

    #[test]
    fn no_materials_means_no_colors_rather_than_a_default_run() {
        // Plain parse ignores assignments; an unknown material resolves to
        // nothing; scaling carries whatever there was.
        assert!(parse_obj(CUBE_OBJ).unwrap().face_colors.is_empty());
        let orphan = parse_obj_with_materials(CUBE_OBJ, &Default::default()).unwrap();
        assert!(orphan.face_colors.is_empty());

        let colored = parse_obj_with_materials(CUBE_OBJ, &palette()).unwrap();
        assert_eq!(
            colored.scaled([2.0, 2.0, 2.0]).face_colors,
            colored.face_colors
        );
    }

    #[test]
    fn faces_before_any_usemtl_take_the_neutral() {
        // Dropping them would misalign every later face against `indices`.
        let obj = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3\nusemtl yellow\nf 1 3 4\n";
        let mesh = parse_obj_with_materials(obj, &palette()).unwrap();
        assert_eq!(mesh.face_colors, vec![NEUTRAL_DIFFUSE, [1.0, 1.0, 0.0]]);
    }
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
        let max_z = parsed
            .vertices
            .iter()
            .map(|v| v[2])
            .fold(f64::MIN, f64::max);
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
