//! nalgebra <-> parry(glam) conversion and URDF geometry -> parry shapes.

use botrail_model::{Geometry, Shape};
use nalgebra::Isometry3;
use parry3d_f64::math::{Pose, Vector};
use parry3d_f64::shape::SharedShape;

use crate::CollideError;

pub fn to_parry_pose(iso: &Isometry3<f64>) -> Pose {
    let t = iso.translation;
    let q = iso.rotation.coords;
    Pose {
        rotation: parry3d_f64::math::Rotation::from_xyzw(q.x, q.y, q.z, q.w),
        translation: Vector::new(t.x, t.y, t.z),
    }
}

/// Maps a URDF geometry to a solid parry shape plus a local offset pose.
/// URDF cylinders extend along +z while parry's extend along +y, hence the
/// x-axis rotation for cylinders.
pub fn geometry_to_parry(geometry: &Geometry) -> Result<(Pose, SharedShape), CollideError> {
    match geometry {
        Geometry::Box { size } => Ok((
            Pose::identity(),
            SharedShape::cuboid(size.x / 2.0, size.y / 2.0, size.z / 2.0),
        )),
        Geometry::Sphere { radius } => Ok((Pose::identity(), SharedShape::ball(*radius))),
        Geometry::Cylinder { radius, length } => {
            let offset = Pose {
                rotation: parry3d_f64::math::Rotation::from_rotation_x(std::f64::consts::FRAC_PI_2),
                translation: Vector::ZERO,
            };
            Ok((offset, SharedShape::cylinder(length / 2.0, *radius)))
        }
        Geometry::Mesh { path, .. } => Err(CollideError::UnsupportedGeometry(format!(
            "mesh `{}` (mesh collision arrives with the mesh I/O crate; use primitives or omit)",
            path.display()
        ))),
    }
}

/// Converts a link-local shape (origin + geometry).
pub fn shape_to_parry(shape: &Shape) -> Result<(Pose, SharedShape), CollideError> {
    let (offset, parry_shape) = geometry_to_parry(&shape.geometry)?;
    Ok((to_parry_pose(&shape.origin) * offset, parry_shape))
}
