//! Baked cloth: the vertex track a cloth pass attaches to a timeline.
//!
//! The simulation lives outside this crate (`botrail-cloth`, whose solver
//! needs native threads). A timeline carries only its result as plain data,
//! so playback, export and the browser build read it without the solver.

use std::collections::BTreeMap;

/// One simulated cloth over a baked cycle: the world position of every
/// vertex at increasing sample times, on the cloth's own clock (a step of
/// its solver, not the playback grid). A cloth at rest keeps only the two
/// ends of the rest span.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClothTrack {
    pub name: String,
    /// Triangles over the vertex indices.
    pub triangles: Vec<[u32; 3]>,
    /// Sample times (s), strictly increasing from 0.
    pub times: Vec<f64>,
    /// `points[k][v]`: world position (Z-up, metres) of vertex `v` at
    /// `times[k]`.
    pub points: Vec<Vec<[f32; 3]>>,
    /// `held[k]`: the vertices a gripper holds at `times[k]`, ascending.
    pub held: Vec<Vec<u32>>,
    /// Named vertices: a garment's cuffs, hem and shoulders, a sheet's
    /// corners.
    pub landmarks: BTreeMap<String, u32>,
    /// Set when the simulation stopped before the end of the cycle: the
    /// time it reached and why. The track ends there and playback holds
    /// its last sample.
    pub failure: Option<(f64, String)>,
    /// What the pass noticed without stopping — a gripper that closed on
    /// nothing, a step it had to split.
    pub warnings: Vec<String>,
}

impl ClothTrack {
    /// The samples around `t`: index `k` and blend `u` such that the state
    /// is `(1 - u) * points[k] + u * points[k + 1]`, clamped to the track.
    fn bracket(&self, t: f64) -> Option<(usize, f64)> {
        let last = self.times.len().checked_sub(1)?;
        if last == 0 || t <= self.times[0] {
            return Some((0, 0.0));
        }
        if t >= self.times[last] {
            return Some((last, 0.0));
        }
        // First sample after `t`; its predecessor is at or before `t`.
        let next = self.times.partition_point(|&s| s <= t);
        let (t0, t1) = (self.times[next - 1], self.times[next]);
        Some((next - 1, (t - t0) / (t1 - t0)))
    }

    /// Vertex positions at `t`, blended linearly between the two samples
    /// around it (the first or last sample outside the track). Empty for a
    /// track without samples.
    pub fn positions_at(&self, t: f64) -> Vec<[f32; 3]> {
        let Some((k, u)) = self.bracket(t) else {
            return Vec::new();
        };
        if u == 0.0 {
            return self.points[k].clone();
        }
        let u = u as f32;
        self.points[k]
            .iter()
            .zip(&self.points[k + 1])
            .map(|(a, b)| {
                [
                    a[0] + (b[0] - a[0]) * u,
                    a[1] + (b[1] - a[1]) * u,
                    a[2] + (b[2] - a[2]) * u,
                ]
            })
            .collect()
    }

    /// The vertices held at `t`: those of the sample at or before it.
    pub fn held_at(&self, t: f64) -> &[u32] {
        match self.bracket(t) {
            Some((k, _)) => self.held.get(k).map_or(&[], Vec::as_slice),
            None => &[],
        }
    }

    /// Drops the interior samples of every span in which nothing changed —
    /// positions and held vertices alike — so a cloth lying still costs two
    /// samples however long it lies. Blending between the kept ends gives
    /// the same state, so playback is unchanged.
    pub fn fold_holds(&mut self) {
        let n = self.times.len();
        if n < 3 {
            return;
        }
        let same = |a: usize, b: usize| {
            self.points[a] == self.points[b] && self.held.get(a) == self.held.get(b)
        };
        let keep: Vec<bool> = (0..n)
            .map(|k| k == 0 || k == n - 1 || !(same(k - 1, k) && same(k, k + 1)))
            .collect();
        let mut flags = keep.iter();
        self.times.retain(|_| *flags.next().unwrap());
        let mut flags = keep.iter();
        self.points.retain(|_| *flags.next().unwrap());
        if self.held.len() == n {
            let mut flags = keep.iter();
            self.held.retain(|_| *flags.next().unwrap());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn track(times: &[f64], heights: &[f32]) -> ClothTrack {
        ClothTrack {
            name: "sheet".into(),
            triangles: vec![[0, 1, 2]],
            times: times.to_vec(),
            points: heights
                .iter()
                .map(|&z| vec![[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]])
                .collect(),
            held: heights.iter().map(|_| Vec::new()).collect(),
            ..ClothTrack::default()
        }
    }

    #[test]
    fn positions_blend_between_samples_and_clamp_outside() {
        let mut t = track(&[0.0, 0.1, 0.3], &[0.0, 1.0, 3.0]);
        t.held[1] = vec![2];
        assert_eq!(t.positions_at(-1.0)[0], [0.0, 0.0, 0.0]);
        assert_eq!(t.positions_at(0.05)[1], [1.0, 0.0, 0.5]);
        assert_eq!(t.positions_at(0.2)[2], [0.0, 1.0, 2.0]);
        assert_eq!(t.positions_at(9.0)[0], [0.0, 0.0, 3.0]);
        // Held vertices step: the sample at or before `t` decides.
        assert!(t.held_at(0.05).is_empty());
        assert_eq!(t.held_at(0.1), [2]);
        assert_eq!(t.held_at(0.29), [2]);
        assert!(t.held_at(0.3).is_empty());
        assert!(ClothTrack::default().positions_at(0.0).is_empty());
        assert!(ClothTrack::default().held_at(0.0).is_empty());
    }

    #[test]
    fn resting_spans_fold_to_their_ends_without_changing_playback() {
        let times: Vec<f64> = (0..8).map(|k| k as f64 * 0.1).collect();
        let heights = [0.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0];
        let full = track(&times, &heights);
        let mut folded = full.clone();
        folded.fold_holds();
        assert_eq!(
            folded.times,
            [times[0], times[1], times[4], times[5], times[7]]
        );
        assert_eq!(folded.points.len(), 5);
        assert_eq!(folded.held.len(), 5);
        for k in 0..=70 {
            let t = k as f64 * 0.01;
            let (a, b) = (full.positions_at(t), folded.positions_at(t));
            for (p, q) in a.iter().zip(&b) {
                assert!((p[2] - q[2]).abs() < 1e-5, "t {t}: {p:?} vs {q:?}");
            }
        }
        // A change of grip alone ends a rest span.
        let mut gripped = track(&times[..5], &[1.0; 5]);
        gripped.held[2] = vec![0];
        gripped.fold_holds();
        assert_eq!(gripped.times, times[..5]);
    }
}
