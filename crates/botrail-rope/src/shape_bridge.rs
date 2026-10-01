//! Extract old Parry shapes into owned dimensions/vertices/indices first.
//! Reconstruction accepts only this neutral data, never old handles or shapes.
use crate::{Error, PoseData};
use parry3d_f64::shape::{SharedShape as OldShape, TypedShape};
use rapier_rope::rapier::prelude::{SharedShape, Vector};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ShapeData {
    Ball(f64),
    Cuboid([f64; 3]),
    Capsule {
        a: [f64; 3],
        b: [f64; 3],
        radius: f64,
    },
    Cylinder {
        half_height: f64,
        radius: f64,
    },
    Cone {
        half_height: f64,
        radius: f64,
    },
    HalfSpace([f64; 3]),
    Triangle([[f64; 3]; 3]),
    TriMesh {
        vertices: Vec<[f64; 3]>,
        indices: Vec<[u32; 3]>,
    },
    /// Exact convex surface triangles, not a new decomposition or AABB.
    ConvexMesh {
        vertices: Vec<[f64; 3]>,
        indices: Vec<[u32; 3]>,
    },
    Compound(Vec<(PoseData, ShapeData)>),
}

pub fn extract(shape: &OldShape) -> Result<ShapeData, Error> {
    let p = |v: parry3d_f64::math::Vector| [v.x, v.y, v.z];
    Ok(match shape.as_typed_shape() {
        TypedShape::Ball(s) => ShapeData::Ball(s.radius),
        TypedShape::Cuboid(s) => ShapeData::Cuboid(p(s.half_extents)),
        TypedShape::Capsule(s) => ShapeData::Capsule {
            a: p(s.segment.a),
            b: p(s.segment.b),
            radius: s.radius,
        },
        TypedShape::Cylinder(s) => ShapeData::Cylinder {
            half_height: s.half_height,
            radius: s.radius,
        },
        TypedShape::Cone(s) => ShapeData::Cone {
            half_height: s.half_height,
            radius: s.radius,
        },
        TypedShape::HalfSpace(s) => ShapeData::HalfSpace(p(s.normal)),
        TypedShape::Triangle(s) => ShapeData::Triangle([p(s.a), p(s.b), p(s.c)]),
        TypedShape::TriMesh(s) => ShapeData::TriMesh {
            vertices: s.vertices().iter().copied().map(p).collect(),
            indices: s.indices().to_vec(),
        },
        TypedShape::ConvexPolyhedron(s) => {
            let (v, i) = s.to_trimesh();
            ShapeData::ConvexMesh {
                vertices: v.into_iter().map(p).collect(),
                indices: i,
            }
        }
        TypedShape::Compound(s) => ShapeData::Compound(
            s.shapes()
                .iter()
                .map(|(pose, s)| Ok((PoseData::from_old(pose), extract(s)?)))
                .collect::<Result<_, Error>>()?,
        ),
        _ => {
            return Err(Error::Input(format!(
                "unsupported Parry 0.29 shape {:?}; no approximation",
                shape.shape_type()
            )))
        }
    })
}

fn positive(x: f64) -> Result<f64, Error> {
    if x.is_finite() && x > 0.0 {
        Ok(x)
    } else {
        Err(Error::Input(
            "shape dimensions must be finite and positive".into(),
        ))
    }
}
fn vector(p: [f64; 3]) -> Result<Vector, Error> {
    if p.into_iter().all(f64::is_finite) {
        Ok(Vector::from_array(p))
    } else {
        Err(Error::Input("non-finite shape vertex or normal".into()))
    }
}
pub fn reconstruct(shape: &ShapeData) -> Result<SharedShape, Error> {
    Ok(match shape {
        ShapeData::Ball(r) => SharedShape::ball(positive(*r)?),
        ShapeData::Cuboid(h) => {
            SharedShape::cuboid(positive(h[0])?, positive(h[1])?, positive(h[2])?)
        }
        ShapeData::Capsule { a, b, radius } => {
            SharedShape::capsule(vector(*a)?, vector(*b)?, positive(*radius)?)
        }
        ShapeData::Cylinder {
            half_height,
            radius,
        } => SharedShape::cylinder(positive(*half_height)?, positive(*radius)?),
        ShapeData::Cone {
            half_height,
            radius,
        } => SharedShape::cone(positive(*half_height)?, positive(*radius)?),
        ShapeData::HalfSpace(n) => {
            let n = vector(*n)?;
            if n.length_squared() < 1e-24 {
                return Err(Error::Input("zero halfspace normal".into()));
            }
            SharedShape::halfspace(n.normalize())
        }
        ShapeData::Triangle(p) => {
            SharedShape::triangle(vector(p[0])?, vector(p[1])?, vector(p[2])?)
        }
        ShapeData::TriMesh { vertices, indices } | ShapeData::ConvexMesh { vertices, indices } => {
            if vertices.is_empty()
                || indices.is_empty()
                || indices
                    .iter()
                    .flatten()
                    .any(|&i| i as usize >= vertices.len())
            {
                return Err(Error::Input(
                    "empty mesh or out-of-range triangle index".into(),
                ));
            }
            let v = vertices
                .iter()
                .copied()
                .map(vector)
                .collect::<Result<Vec<_>, _>>()?;
            if matches!(shape, ShapeData::ConvexMesh { .. }) {
                SharedShape::convex_mesh(v, indices)
                    .ok_or_else(|| Error::Input("invalid convex surface".into()))?
            } else {
                SharedShape::trimesh(v, indices.clone())
                    .map_err(|e| Error::Input(format!("triangle mesh: {e}")))?
            }
        }
        ShapeData::Compound(parts) => {
            if parts.is_empty() {
                return Err(Error::Input("empty compound".into()));
            }
            if parts
                .iter()
                .any(|(_, s)| matches!(s, ShapeData::Compound(_) | ShapeData::TriMesh { .. }))
            {
                return Err(Error::Input(
                    "native compound cannot contain composite shapes; no approximation".into(),
                ));
            }
            SharedShape::compound(
                parts
                    .iter()
                    .map(|(p, s)| Ok((p.native()?, reconstruct(s)?)))
                    .collect::<Result<_, Error>>()?,
            )
        }
    })
}
