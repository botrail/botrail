//! Baked one-way rope replay in a private Rapier 0.36 f64 world.
//! All inputs/outputs use botrail's right-handed Z-up SI frame. No source
//! physics world is borrowed or mutated and no reaction is returned to it.
pub mod pass;
pub mod shape_bridge;
pub mod track;

pub use botrail_scene::rope::{RopeConnectorTrack, RopeEvent, RopeTrack};
use rapier_rope::rapier::prelude::{Pose, Rotation, Vector};
pub use rapier_rope::{RopeLocation, RopeSpec};
use serde::{Deserialize, Serialize};
/// Definition types for callers constructing a replay pass.
pub mod rapier_rope_types {
    pub use rapier_rope::{
        CollisionSettings, NativeRopeMaterial, RopeSpec, SamplingSettings, SpringSettings,
    };
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("rope replay input: {0}")]
    Input(String),
    #[error(transparent)]
    Registry(#[from] rapier_rope::RopeSetError),
    #[error(transparent)]
    Output(#[from] rapier_rope::OutputError),
    /// Native step already advanced; callers must not retry this world.
    #[error(
        "rope world advanced to {time_s}s before validation failed; no rollback or retry: {reason}"
    )]
    Advanced { time_s: f64, reason: String },
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PoseData {
    pub position: [f64; 3],
    pub quaternion: [f64; 4],
}
impl Default for PoseData {
    fn default() -> Self {
        Self {
            position: [0.0; 3],
            quaternion: [0.0, 0.0, 0.0, 1.0],
        }
    }
}
impl PoseData {
    pub fn from_iso(p: &nalgebra::Isometry3<f64>) -> Self {
        let q = p.rotation.quaternion();
        Self {
            position: [p.translation.x, p.translation.y, p.translation.z],
            quaternion: [q.i, q.j, q.k, q.w],
        }
    }
    pub fn from_old(p: &parry3d_f64::math::Pose) -> Self {
        Self {
            position: p.translation.into(),
            quaternion: p.rotation.to_array(),
        }
    }
    pub fn from_native(p: &Pose) -> Self {
        Self {
            position: p.translation.into(),
            quaternion: p.rotation.to_array(),
        }
    }
    pub fn native(&self) -> Result<Pose, Error> {
        let norm: f64 = self.quaternion.iter().map(|v| v * v).sum();
        if !self.position.into_iter().all(f64::is_finite)
            || !norm.is_finite()
            || (norm - 1.0).abs() > 1e-8
        {
            return Err(Error::Input(
                "pose must be finite with a unit quaternion".into(),
            ));
        }
        Ok(Pose {
            translation: Vector::from_array(self.position),
            rotation: Rotation::from_array(self.quaternion),
        })
    }
    /// Same shortest-arc normalized-linear quaternion blend as Studio's
    /// baked link poses (not FK of interpolated joints, and no axis swap).
    pub fn interpolate(self, b: Self, u: f64) -> Self {
        let sign = if self
            .quaternion
            .iter()
            .zip(b.quaternion)
            .map(|(a, b)| a * b)
            .sum::<f64>()
            < 0.0
        {
            -1.0
        } else {
            1.0
        };
        let mut q: [f64; 4] = std::array::from_fn(|i| {
            self.quaternion[i] + (sign * b.quaternion[i] - self.quaternion[i]) * u
        });
        let norm = q.iter().map(|x| x * x).sum::<f64>().sqrt();
        for x in &mut q {
            *x /= norm;
        }
        Self {
            position: std::array::from_fn(|i| {
                self.position[i] + (b.position[i] - self.position[i]) * u
            }),
            quaternion: q,
        }
    }
    pub fn message(self) -> botrail_scene::wire::PoseMsg {
        botrail_scene::wire::PoseMsg {
            position: self.position,
            quaternion: self.quaternion,
        }
    }
}
