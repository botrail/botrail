//! Playback-only rope data. No Rapier types or solver dependency cross this boundary.
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct RopeEvent {
    pub signal: String,
    pub body: String,
    pub closed: bool,
    /// Exact baked signal time, including pulses shorter than a nominal step.
    pub source_time_s: f64,
    /// The pass splits its clock at the source time; no quantization.
    pub applied_time_s: f64,
    pub particle: u32,
    /// Captured in the link frame at acquisition; no teleport or orientation clamp.
    pub local_anchor_m: Option<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct RopeConnectorTrack {
    pub name: String,
    /// Sphere: one radius; cuboid: three half extents, in metres.
    pub shape: String,
    pub dimensions_m: Vec<f64>,
    pub mass_kg: f64,
    /// Same clock as the centerline, including its exact final sample.
    pub poses: Vec<crate::wire::PoseMsg>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct RopeTrack {
    pub name: String,
    pub segments: Vec<[u32; 2]>,
    pub radius_m: f64,
    pub times: Vec<f64>,
    /// Z-up, right handed, SI; f64 storage, f32 GPU display buffers only.
    pub points: Vec<Vec<[f64; 3]>>,
    /// Attachments after events at each sample; step function in playback.
    pub held: Vec<Vec<u32>>,
    pub events: Vec<RopeEvent>,
    pub connectors: Vec<RopeConnectorTrack>,
    /// Always `baked_one_way_no_source_reaction`; never a live solver checkpoint.
    pub coupling: String,
    pub coordinate_system: String,
    pub solver: String,
}

impl RopeTrack {
    pub fn positions_at(&self, t: f64) -> Vec<[f64; 3]> {
        if self.times.is_empty() {
            return Vec::new();
        }
        let k = self.times.partition_point(|s| *s <= t).saturating_sub(1);
        let j = (k + 1).min(self.times.len() - 1);
        let u = if k == j {
            0.0
        } else {
            ((t - self.times[k]) / (self.times[j] - self.times[k])).clamp(0.0, 1.0)
        };
        self.points[k]
            .iter()
            .zip(&self.points[j])
            .map(|(a, b)| std::array::from_fn(|i| a[i] + (b[i] - a[i]) * u))
            .collect()
    }
    pub fn held_at(&self, t: f64) -> &[u32] {
        if self.times.is_empty() {
            return &[];
        }
        &self.held[self.times.partition_point(|s| *s <= t).saturating_sub(1)]
    }
}
