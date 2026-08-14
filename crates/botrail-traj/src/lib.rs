//! Time parameterization of joint-space paths.
//!
//! The algorithm is an iterative parabolic scheme (in the spirit of
//! MoveIt's IPTP): segment durations start at the per-joint velocity bound
//! and are stretched until interface accelerations — including a rest-to-
//! rest boundary at both ends — respect the acceleration bound. Sampling
//! between waypoints is cubic Hermite on the waypoint velocities, so the
//! preview and exports are smooth. Jerk limits arrive post-M3 (Ruckig-style
//! is on the roadmap).

use thiserror::Error;

#[derive(Debug, Error)]
pub enum TrajError {
    #[error("path is empty")]
    EmptyPath,
    #[error("expected {expected} joint values, got {got} at waypoint {index}")]
    WrongDof {
        expected: usize,
        got: usize,
        index: usize,
    },
    #[error("velocity/acceleration limits must be positive")]
    NonPositiveLimits,
    #[error("expected {expected} segment duration floors, got {got}")]
    WrongFloorCount { expected: usize, got: usize },
}

/// Per-joint velocity and acceleration bounds (absolute values).
#[derive(Debug, Clone)]
pub struct Limits {
    pub velocity: Vec<f64>,
    pub acceleration: Vec<f64>,
}

impl Limits {
    pub fn uniform(dof: usize, velocity: f64, acceleration: f64) -> Self {
        Limits {
            velocity: vec![velocity; dof],
            acceleration: vec![acceleration; dof],
        }
    }
}

#[derive(Debug, Clone)]
pub struct TimingOptions {
    /// Input segments longer than this (joint-space L2) are subdivided
    /// before timing, so accelerations are controlled along the whole path.
    pub max_segment: f64,
    /// Acceleration-adjustment sweeps.
    pub max_passes: usize,
}

impl Default for TimingOptions {
    fn default() -> Self {
        TimingOptions {
            max_segment: 0.15,
            max_passes: 64,
        }
    }
}

/// A time-parameterized joint trajectory (waypoints + waypoint velocities).
#[derive(Debug, Clone)]
pub struct JointTrajectory {
    pub times: Vec<f64>,
    pub positions: Vec<Vec<f64>>,
    pub velocities: Vec<Vec<f64>>,
}

impl JointTrajectory {
    pub fn dof(&self) -> usize {
        self.positions.first().map_or(0, Vec::len)
    }

    pub fn duration(&self) -> f64 {
        *self.times.last().unwrap_or(&0.0)
    }

    /// Cubic-Hermite sample at time `t` (clamped to the trajectory span).
    pub fn sample(&self, t: f64) -> Vec<f64> {
        let n = self.times.len();
        if n == 1 || t <= self.times[0] {
            return self.positions[0].clone();
        }
        if t >= self.duration() {
            return self.positions[n - 1].clone();
        }
        let seg = match self
            .times
            .binary_search_by(|probe| probe.partial_cmp(&t).unwrap())
        {
            Ok(i) => return self.positions[i].clone(),
            Err(i) => i - 1,
        };
        let h = self.times[seg + 1] - self.times[seg];
        let u = (t - self.times[seg]) / h;
        let (u2, u3) = (u * u, u * u * u);
        let (h00, h10, h01, h11) = (
            2.0 * u3 - 3.0 * u2 + 1.0,
            u3 - 2.0 * u2 + u,
            -2.0 * u3 + 3.0 * u2,
            u3 - u2,
        );
        (0..self.dof())
            .map(|j| {
                h00 * self.positions[seg][j]
                    + h10 * h * self.velocities[seg][j]
                    + h01 * self.positions[seg + 1][j]
                    + h11 * h * self.velocities[seg + 1][j]
            })
            .collect()
    }

    /// Uniformly resampled copy at period `dt` (always includes the final
    /// configuration).
    pub fn resample(&self, dt: f64) -> (Vec<f64>, Vec<Vec<f64>>) {
        let duration = self.duration();
        let mut times = Vec::new();
        let mut positions = Vec::new();
        let mut t = 0.0;
        while t < duration {
            times.push(t);
            positions.push(self.sample(t));
            t += dt;
        }
        times.push(duration);
        positions.push(self.positions.last().cloned().unwrap_or_default());
        (times, positions)
    }
}

fn distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

/// Densified copy of `path`, with per-output-segment duration floors
/// (an input interval's floor is split evenly across its subdivisions) and
/// the output index of each input waypoint. Zero-length input intervals
/// collapse — their floor, if any, is dropped with them (a dwell is not
/// expressible as a duplicated waypoint here).
fn densify(
    path: &[Vec<f64>],
    max_segment: f64,
    floors: Option<&[f64]>,
) -> (Vec<Vec<f64>>, Vec<f64>, Vec<usize>) {
    let mut out = vec![path[0].clone()];
    let mut seg_floors = Vec::new();
    let mut indices = vec![0usize];
    for (i, w) in path.windows(2).enumerate() {
        let d = distance(&w[0], &w[1]);
        if d < 1e-12 {
            indices.push(out.len() - 1);
            continue;
        }
        let steps = (d / max_segment).ceil().max(1.0) as usize;
        let floor = floors.map_or(0.0, |f| f[i] / steps as f64);
        for k in 1..steps {
            let t = k as f64 / steps as f64;
            out.push(
                w[0].iter()
                    .zip(&w[1])
                    .map(|(a, b)| a + (b - a) * t)
                    .collect(),
            );
            seg_floors.push(floor);
        }
        // Push the exact endpoint (lerp at t=1 would round).
        out.push(w[1].clone());
        seg_floors.push(floor);
        indices.push(out.len() - 1);
    }
    (out, seg_floors, indices)
}

/// Assigns times to a joint-space path so that per-joint velocity and
/// acceleration bounds hold at the waypoint representation.
pub fn time_parameterize(
    path: &[Vec<f64>],
    limits: &Limits,
    options: &TimingOptions,
) -> Result<JointTrajectory, TrajError> {
    Ok(parameterize_impl(path, limits, None, options)?.trajectory)
}

/// A timed path plus the mapping back to its input waypoints.
#[derive(Debug, Clone)]
pub struct FloorTiming {
    pub trajectory: JointTrajectory,
    /// For each input waypoint, its index into `trajectory` waypoints
    /// (densification inserts points; duplicate inputs collapse).
    pub waypoint_indices: Vec<usize>,
}

/// [`time_parameterize`] with a lower bound on each input interval's
/// duration (seconds; `0.0` = no floor). This is how a Cartesian feed rate
/// becomes joint timing: the caller computes `chord length / feed` per
/// interval, and joints only slow the path further where their own
/// velocity or acceleration bounds demand it — never speed it up.
pub fn time_parameterize_with_floors(
    path: &[Vec<f64>],
    limits: &Limits,
    min_durations: &[f64],
    options: &TimingOptions,
) -> Result<FloorTiming, TrajError> {
    let expected = path.len().saturating_sub(1);
    if min_durations.len() != expected {
        return Err(TrajError::WrongFloorCount {
            expected,
            got: min_durations.len(),
        });
    }
    parameterize_impl(path, limits, Some(min_durations), options)
}

fn parameterize_impl(
    path: &[Vec<f64>],
    limits: &Limits,
    floors: Option<&[f64]>,
    options: &TimingOptions,
) -> Result<FloorTiming, TrajError> {
    if path.is_empty() {
        return Err(TrajError::EmptyPath);
    }
    let dof = path[0].len();
    for (index, q) in path.iter().enumerate() {
        if q.len() != dof {
            return Err(TrajError::WrongDof {
                expected: dof,
                got: q.len(),
                index,
            });
        }
    }
    if limits.velocity.len() != dof
        || limits.acceleration.len() != dof
        || limits.velocity.iter().any(|v| *v <= 0.0)
        || limits.acceleration.iter().any(|a| *a <= 0.0)
    {
        return Err(TrajError::NonPositiveLimits);
    }

    let (points, seg_floors, waypoint_indices) = densify(path, options.max_segment, floors);
    let n = points.len();
    if n == 1 {
        return Ok(FloorTiming {
            trajectory: JointTrajectory {
                times: vec![0.0],
                positions: points,
                velocities: vec![vec![0.0; dof]],
            },
            waypoint_indices,
        });
    }

    // 1. Velocity-bound initial durations, lifted onto the floors.
    let nseg = n - 1;
    let mut dt = vec![0.0f64; nseg];
    for (i, w) in points.windows(2).enumerate() {
        let mut min_dt: f64 = 1e-4;
        // j indexes three parallel arrays; an iterator chain would obscure it.
        #[allow(clippy::needless_range_loop)]
        for j in 0..dof {
            min_dt = min_dt.max((w[1][j] - w[0][j]).abs() / limits.velocity[j]);
        }
        dt[i] = min_dt.max(seg_floors[i]);
    }

    // 2. Stretch durations until interface accelerations (with rest-to-rest
    //    boundaries) are within bounds.
    for _ in 0..options.max_passes {
        let mut changed = false;
        for i in 0..=nseg {
            // j indexes points/dt/limits in parallel.
            #[allow(clippy::needless_range_loop)]
            for j in 0..dof {
                let v_prev = if i > 0 {
                    (points[i][j] - points[i - 1][j]) / dt[i - 1]
                } else {
                    0.0
                };
                let v_next = if i < nseg {
                    (points[i + 1][j] - points[i][j]) / dt[i]
                } else {
                    0.0
                };
                // Conservative span: the shorter adjacent segment. The
                // average span makes the discrete estimate optimistic vs
                // the continuous profile; min biases toward stretching.
                let span = match (i > 0, i < nseg) {
                    (true, true) => dt[i - 1].min(dt[i]),
                    (true, false) => dt[i - 1],
                    (false, true) => dt[i],
                    (false, false) => unreachable!("n >= 2"),
                };
                let acc = (v_next - v_prev) / span;
                if acc.abs() > limits.acceleration[j] * 1.001 {
                    // Stretching by sqrt(ratio) halves the acceleration
                    // roughly quadratically; cap the growth per pass.
                    let scale = (acc.abs() / limits.acceleration[j]).sqrt().min(1.5);
                    if i > 0 {
                        dt[i - 1] *= scale;
                    }
                    if i < nseg {
                        dt[i] *= scale;
                    }
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }

    // 3. Timestamps and waypoint velocities (central differences, rest at
    //    the endpoints), clamped into the velocity bounds.
    let mut times = Vec::with_capacity(n);
    let mut acc_t = 0.0;
    times.push(0.0);
    for d in &dt {
        acc_t += d;
        times.push(acc_t);
    }
    let mut velocities = vec![vec![0.0; dof]; n];
    for i in 1..n - 1 {
        for j in 0..dof {
            let v = (points[i + 1][j] - points[i - 1][j]) / (times[i + 1] - times[i - 1]);
            velocities[i][j] = v.clamp(-limits.velocity[j], limits.velocity[j]);
        }
    }

    Ok(FloorTiming {
        trajectory: JointTrajectory {
            times,
            positions: points,
            velocities,
        },
        waypoint_indices,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits1() -> Limits {
        Limits::uniform(1, 1.0, 1.0)
    }

    #[test]
    fn single_joint_move_respects_limits() {
        let path = vec![vec![0.0], vec![1.0]];
        let traj = time_parameterize(&path, &limits1(), &TimingOptions::default()).unwrap();

        // Endpoints preserved.
        assert_eq!(traj.positions.first().unwrap(), &vec![0.0]);
        assert_eq!(traj.positions.last().unwrap(), &vec![1.0]);
        // Strictly increasing times.
        assert!(traj.times.windows(2).all(|w| w[1] > w[0]));
        // Segment velocities within bound.
        for (w, d) in traj
            .positions
            .windows(2)
            .zip(traj.times.windows(2).map(|w| w[1] - w[0]))
        {
            assert!(((w[1][0] - w[0][0]) / d).abs() <= 1.0 + 1e-9);
        }
        // Interface accelerations within bound (incl. rest boundaries).
        let n = traj.times.len();
        for i in 0..n {
            let v_prev = if i > 0 {
                (traj.positions[i][0] - traj.positions[i - 1][0])
                    / (traj.times[i] - traj.times[i - 1])
            } else {
                0.0
            };
            let v_next = if i < n - 1 {
                (traj.positions[i + 1][0] - traj.positions[i][0])
                    / (traj.times[i + 1] - traj.times[i])
            } else {
                0.0
            };
            let span = if i == 0 {
                traj.times[1] - traj.times[0]
            } else if i == n - 1 {
                traj.times[n - 1] - traj.times[n - 2]
            } else {
                (traj.times[i] - traj.times[i - 1]).min(traj.times[i + 1] - traj.times[i])
            };
            assert!(
                ((v_next - v_prev) / span).abs() <= 1.0 * 1.01 + 1e-9,
                "acc violated at waypoint {i}"
            );
        }
        // A 1-rad rest-to-rest move with v=a=1 cannot beat the bang-bang
        // optimum of 2s; it also should not be pathologically slow.
        let d = traj.duration();
        assert!((1.9..6.0).contains(&d), "duration = {d}");
    }

    #[test]
    fn sampling_is_monotone_and_hits_endpoints() {
        let path = vec![vec![0.0, 0.0], vec![0.5, -0.3], vec![1.0, 0.4]];
        let limits = Limits::uniform(2, 2.0, 4.0);
        let traj = time_parameterize(&path, &limits, &TimingOptions::default()).unwrap();
        assert_eq!(traj.sample(-1.0), vec![0.0, 0.0]);
        assert_eq!(traj.sample(traj.duration() + 1.0), vec![1.0, 0.4]);
        // Hermite samples stay near the waypoint hull (no wild overshoot).
        let (_, samples) = traj.resample(0.01);
        for q in samples {
            assert!(q[0] >= -0.05 && q[0] <= 1.05);
            assert!(q[1] >= -0.4 && q[1] <= 0.5);
        }
    }

    #[test]
    fn resample_includes_final_point() {
        let path = vec![vec![0.0], vec![0.4]];
        let traj = time_parameterize(&path, &limits1(), &TimingOptions::default()).unwrap();
        let (times, positions) = traj.resample(0.033);
        assert_eq!(times.first(), Some(&0.0));
        assert!((times.last().unwrap() - traj.duration()).abs() < 1e-12);
        assert_eq!(positions.last().unwrap(), &vec![0.4]);
    }

    #[test]
    fn floors_hold_when_joints_could_go_faster() {
        // 0.1 rad with v=1 would take ~0.1s; the 2s floor must win, and the
        // resulting motion is so gentle the acceleration pass leaves it be.
        let path = vec![vec![0.0], vec![0.1]];
        let timed =
            time_parameterize_with_floors(&path, &limits1(), &[2.0], &TimingOptions::default())
                .unwrap();
        assert!((timed.trajectory.duration() - 2.0).abs() < 1e-9);
        assert_eq!(timed.waypoint_indices, vec![0, 1]);
    }

    #[test]
    fn joint_limits_stretch_past_an_ambitious_floor() {
        // A 1-rad move floored at 10ms: the velocity bound (1 rad/s) makes
        // that impossible, so the joint-based timing must win.
        let path = vec![vec![0.0], vec![1.0]];
        let timed =
            time_parameterize_with_floors(&path, &limits1(), &[0.01], &TimingOptions::default())
                .unwrap();
        assert!(timed.trajectory.duration() >= 1.0);
    }

    #[test]
    fn floors_survive_densification() {
        // 0.3 rad splits into two sub-segments at max_segment 0.15; the 1s
        // floor is shared between them and the total still comes out to 1s.
        let path = vec![vec![0.0], vec![0.3]];
        let limits = Limits::uniform(1, 10.0, 10.0);
        let timed =
            time_parameterize_with_floors(&path, &limits, &[1.0], &TimingOptions::default())
                .unwrap();
        assert!((timed.trajectory.duration() - 1.0).abs() < 1e-9);
        assert_eq!(timed.trajectory.times.len(), 3);
        assert_eq!(timed.waypoint_indices, vec![0, 2]);
    }

    #[test]
    fn zero_floors_match_the_legacy_path_bit_for_bit() {
        let path = vec![
            vec![0.0, 0.0],
            vec![0.5, -0.3],
            vec![0.5, -0.3],
            vec![1.0, 0.4],
        ];
        let limits = Limits::uniform(2, 2.0, 4.0);
        let legacy = time_parameterize(&path, &limits, &TimingOptions::default()).unwrap();
        let floored =
            time_parameterize_with_floors(&path, &limits, &[0.0; 3], &TimingOptions::default())
                .unwrap();
        assert_eq!(legacy.times, floored.trajectory.times);
        assert_eq!(legacy.positions, floored.trajectory.positions);
        assert_eq!(legacy.velocities, floored.trajectory.velocities);
        // The duplicated input waypoint collapses onto its twin.
        assert_eq!(floored.waypoint_indices[1], floored.waypoint_indices[2]);
    }

    #[test]
    fn floor_count_mismatch_is_rejected() {
        assert!(matches!(
            time_parameterize_with_floors(
                &[vec![0.0], vec![1.0]],
                &limits1(),
                &[0.1, 0.2],
                &TimingOptions::default()
            ),
            Err(TrajError::WrongFloorCount {
                expected: 1,
                got: 2
            })
        ));
    }

    #[test]
    fn degenerate_paths() {
        assert!(matches!(
            time_parameterize(&[], &limits1(), &TimingOptions::default()),
            Err(TrajError::EmptyPath)
        ));
        let stationary = vec![vec![0.3], vec![0.3]];
        let traj = time_parameterize(&stationary, &limits1(), &TimingOptions::default()).unwrap();
        assert_eq!(traj.times, vec![0.0]);
        assert!(matches!(
            time_parameterize(
                &[vec![0.0], vec![1.0]],
                &Limits::uniform(1, 0.0, 1.0),
                &TimingOptions::default()
            ),
            Err(TrajError::NonPositiveLimits)
        ));
    }
}
