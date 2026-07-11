//! Sampling-based joint-space motion planning: RRT-Connect with random
//! shortcut smoothing.
//!
//! The planner is deliberately decoupled from the robot/scene: it plans in a
//! box-bounded joint space against a caller-provided validity predicate
//! (typically "within limits and collision-free", built from a `Scene`).
//! All randomness is a seeded xorshift, so plans are reproducible.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum PlanError {
    #[error("expected {expected} joint values, got {got}")]
    WrongDof { expected: usize, got: usize },
    #[error("start configuration is invalid (in collision or out of limits)")]
    InvalidStart,
    #[error("goal configuration is invalid (in collision or out of limits)")]
    InvalidGoal,
    #[error("no plan found within {iters} iterations")]
    NotFound { iters: usize },
}

/// Box bounds of the joint space to sample in.
#[derive(Debug, Clone)]
pub struct JointSpace {
    pub lower: Vec<f64>,
    pub upper: Vec<f64>,
}

impl JointSpace {
    pub fn dof(&self) -> usize {
        self.lower.len()
    }

    fn contains(&self, q: &[f64]) -> bool {
        q.iter()
            .zip(self.lower.iter().zip(&self.upper))
            .all(|(v, (lo, hi))| v >= lo && v <= hi)
    }
}

#[derive(Debug, Clone)]
pub struct PlanOptions {
    /// Maximum RRT-Connect iterations (one sample + extend/connect each).
    pub max_iters: usize,
    /// Joint-space L2 norm of a single tree extension.
    pub step_size: f64,
    /// Interpolation resolution (L2 norm) for edge validity checks.
    pub resolution: f64,
    /// Random shortcut smoothing attempts applied to the raw path.
    pub shortcut_iters: usize,
    pub seed: u64,
}

impl Default for PlanOptions {
    fn default() -> Self {
        PlanOptions {
            max_iters: 10_000,
            step_size: 0.3,
            resolution: 0.05,
            shortcut_iters: 200,
            seed: 0x0B07_2A11,
        }
    }
}

struct XorShift(u64);

impl XorShift {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }

    fn below(&mut self, n: usize) -> usize {
        (self.unit() * n as f64) as usize % n
    }
}

fn distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

fn lerp(a: &[f64], b: &[f64], t: f64) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x + (y - x) * t).collect()
}

/// True if the straight segment between two *valid* endpoints stays valid
/// at `resolution` granularity (endpoints are not re-checked).
fn edge_valid(
    a: &[f64],
    b: &[f64],
    resolution: f64,
    is_valid: &mut dyn FnMut(&[f64]) -> bool,
) -> bool {
    let dist = distance(a, b);
    let steps = (dist / resolution).ceil() as usize;
    for k in 1..steps {
        if !is_valid(&lerp(a, b, k as f64 / steps as f64)) {
            return false;
        }
    }
    true
}

struct Node {
    q: Vec<f64>,
    parent: Option<usize>,
}

struct Tree {
    nodes: Vec<Node>,
}

impl Tree {
    fn new(root: Vec<f64>) -> Self {
        Tree {
            nodes: vec![Node {
                q: root,
                parent: None,
            }],
        }
    }

    fn nearest(&self, q: &[f64]) -> usize {
        let mut best = 0;
        let mut best_d = f64::INFINITY;
        for (i, node) in self.nodes.iter().enumerate() {
            let d = distance(&node.q, q);
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        best
    }

    fn add(&mut self, q: Vec<f64>, parent: usize) -> usize {
        self.nodes.push(Node {
            q,
            parent: Some(parent),
        });
        self.nodes.len() - 1
    }

    fn path_to_root(&self, mut index: usize) -> Vec<Vec<f64>> {
        let mut path = Vec::new();
        loop {
            path.push(self.nodes[index].q.clone());
            match self.nodes[index].parent {
                Some(p) => index = p,
                None => break,
            }
        }
        path
    }
}

enum Extend {
    Reached(usize),
    Advanced(usize),
    Trapped,
}

/// One RRT extension of `tree` toward `target` by at most `step_size`.
fn extend(
    tree: &mut Tree,
    target: &[f64],
    options: &PlanOptions,
    is_valid: &mut dyn FnMut(&[f64]) -> bool,
) -> Extend {
    let nearest = tree.nearest(target);
    let q_near = tree.nodes[nearest].q.clone();
    let dist = distance(&q_near, target);
    if dist < 1e-12 {
        return Extend::Reached(nearest);
    }
    let (q_new, reached) = if dist <= options.step_size {
        (target.to_vec(), true)
    } else {
        (lerp(&q_near, target, options.step_size / dist), false)
    };
    if !is_valid(&q_new) || !edge_valid(&q_near, &q_new, options.resolution, is_valid) {
        return Extend::Trapped;
    }
    let index = tree.add(q_new, nearest);
    if reached {
        Extend::Reached(index)
    } else {
        Extend::Advanced(index)
    }
}

/// Greedily extends `tree` toward `target` until reached or blocked.
fn connect(
    tree: &mut Tree,
    target: &[f64],
    options: &PlanOptions,
    is_valid: &mut dyn FnMut(&[f64]) -> bool,
) -> Option<usize> {
    loop {
        match extend(tree, target, options, is_valid) {
            Extend::Reached(i) => return Some(i),
            Extend::Advanced(_) => continue,
            Extend::Trapped => return None,
        }
    }
}

/// Plans a collision-free joint-space path from `start` to `goal`
/// (inclusive at both ends) with RRT-Connect, then applies random shortcut
/// smoothing. The result is a sparse waypoint list; consecutive waypoints
/// are connected by valid straight segments at `options.resolution`.
pub fn plan(
    space: &JointSpace,
    start: &[f64],
    goal: &[f64],
    is_valid: &mut dyn FnMut(&[f64]) -> bool,
    options: &PlanOptions,
) -> Result<Vec<Vec<f64>>, PlanError> {
    let dof = space.dof();
    if start.len() != dof {
        return Err(PlanError::WrongDof {
            expected: dof,
            got: start.len(),
        });
    }
    if goal.len() != dof {
        return Err(PlanError::WrongDof {
            expected: dof,
            got: goal.len(),
        });
    }
    if !space.contains(start) || !is_valid(start) {
        return Err(PlanError::InvalidStart);
    }
    if !space.contains(goal) || !is_valid(goal) {
        return Err(PlanError::InvalidGoal);
    }

    // Trivial case: straight connection.
    if edge_valid(start, goal, options.resolution, is_valid) {
        return Ok(vec![start.to_vec(), goal.to_vec()]);
    }

    let mut rng = XorShift(options.seed | 1);
    let mut tree_a = Tree::new(start.to_vec());
    let mut tree_b = Tree::new(goal.to_vec());
    // `swapped` tracks whether tree_a currently grows from the goal side.
    let mut swapped = false;

    for iter in 0..options.max_iters {
        let sample: Vec<f64> = space
            .lower
            .iter()
            .zip(&space.upper)
            .map(|(lo, hi)| lo + rng.unit() * (hi - lo))
            .collect();

        match extend(&mut tree_a, &sample, options, is_valid) {
            Extend::Trapped => {}
            Extend::Reached(new_index) | Extend::Advanced(new_index) => {
                let q_new = tree_a.nodes[new_index].q.clone();
                if let Some(b_index) = connect(&mut tree_b, &q_new, options, is_valid) {
                    // Join: path from start-side root to q_new, then to
                    // goal-side root.
                    let (start_tree, start_index, goal_tree, goal_index) = if swapped {
                        (&tree_b, b_index, &tree_a, new_index)
                    } else {
                        (&tree_a, new_index, &tree_b, b_index)
                    };
                    let mut path: Vec<Vec<f64>> = start_tree
                        .path_to_root(start_index)
                        .into_iter()
                        .rev()
                        .collect();
                    let goal_path = goal_tree.path_to_root(goal_index);
                    // Both paths include the junction configuration.
                    path.extend(goal_path.into_iter().skip(1));
                    let path = shortcut(path, options, &mut rng, is_valid);
                    let _ = iter;
                    return Ok(path);
                }
            }
        }
        std::mem::swap(&mut tree_a, &mut tree_b);
        swapped = !swapped;
    }
    Err(PlanError::NotFound {
        iters: options.max_iters,
    })
}

/// Random shortcut: repeatedly tries to replace a sub-path with a straight
/// segment. Keeps endpoints; result never gets longer.
fn shortcut(
    mut path: Vec<Vec<f64>>,
    options: &PlanOptions,
    rng: &mut XorShift,
    is_valid: &mut dyn FnMut(&[f64]) -> bool,
) -> Vec<Vec<f64>> {
    for _ in 0..options.shortcut_iters {
        if path.len() < 3 {
            break;
        }
        let i = rng.below(path.len() - 2);
        let j = i + 2 + rng.below(path.len() - i - 2);
        if edge_valid(&path[i], &path[j], options.resolution, is_valid) {
            path.drain(i + 1..j);
        }
    }
    path
}

/// Total joint-space L2 length of a path.
pub fn path_length(path: &[Vec<f64>]) -> f64 {
    path.windows(2).map(|w| distance(&w[0], &w[1])).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn space2() -> JointSpace {
        JointSpace {
            lower: vec![-1.0, -1.0],
            upper: vec![1.0, 1.0],
        }
    }

    /// Disc obstacle of radius 0.4 at the origin of a 2D joint space.
    fn disc_invalid(q: &[f64]) -> bool {
        q[0] * q[0] + q[1] * q[1] > 0.16
    }

    #[test]
    fn trivial_straight_line() {
        let mut ok = |_: &[f64]| true;
        let path = plan(
            &space2(),
            &[-0.9, 0.0],
            &[0.9, 0.0],
            &mut ok,
            &PlanOptions::default(),
        )
        .unwrap();
        assert_eq!(path.len(), 2);
    }

    #[test]
    fn plans_around_disc_and_result_is_valid() {
        // Start and goal on opposite sides of the disc: the straight line
        // is blocked, so the planner must detour around it.
        let mut is_valid = disc_invalid;
        let options = PlanOptions::default();
        let path = plan(
            &space2(),
            &[-0.9, 0.0],
            &[0.9, 0.0],
            &mut is_valid,
            &options,
        )
        .unwrap();

        assert_eq!(path.first().unwrap(), &vec![-0.9, 0.0]);
        assert_eq!(path.last().unwrap(), &vec![0.9, 0.0]);
        // Every densely-interpolated configuration along the path is valid.
        for w in path.windows(2) {
            let steps = (distance(&w[0], &w[1]) / 0.01).ceil() as usize;
            for k in 0..=steps {
                let q = lerp(&w[0], &w[1], k as f64 / steps.max(1) as f64);
                assert!(disc_invalid(&q), "path passes through obstacle at {q:?}");
            }
        }
        // The detour must be longer than the straight line but sane.
        let len = path_length(&path);
        assert!(len > 1.8 && len < 4.0, "len = {len}");
    }

    #[test]
    fn deterministic_for_fixed_seed() {
        let mut v1 = disc_invalid;
        let mut v2 = disc_invalid;
        let options = PlanOptions::default();
        let p1 = plan(&space2(), &[-0.9, 0.0], &[0.9, 0.0], &mut v1, &options).unwrap();
        let p2 = plan(&space2(), &[-0.9, 0.0], &[0.9, 0.0], &mut v2, &options).unwrap();
        assert_eq!(p1, p2);
    }

    #[test]
    fn shortcut_shrinks_paths() {
        // A deliberately wiggly (but valid) path in free space must collapse
        // to (nearly) the straight segment.
        let mut ok = |_: &[f64]| true;
        let wiggly = vec![
            vec![0.0, 0.0],
            vec![0.2, 0.8],
            vec![0.4, -0.8],
            vec![0.6, 0.8],
            vec![1.0, 0.0],
        ];
        let before = path_length(&wiggly);
        let mut rng = XorShift(7);
        let after = shortcut(wiggly, &PlanOptions::default(), &mut rng, &mut ok);
        assert!(path_length(&after) < before * 0.5);
        assert_eq!(after.len(), 2);
    }

    #[test]
    fn invalid_endpoints_are_rejected() {
        let mut is_valid = disc_invalid;
        let options = PlanOptions::default();
        assert!(matches!(
            plan(&space2(), &[0.0, 0.0], &[0.9, 0.0], &mut is_valid, &options),
            Err(PlanError::InvalidStart)
        ));
        assert!(matches!(
            plan(&space2(), &[0.9, 0.0], &[0.1, 0.0], &mut is_valid, &options),
            Err(PlanError::InvalidGoal)
        ));
        assert!(matches!(
            plan(&space2(), &[0.9], &[0.1, 0.0], &mut is_valid, &options),
            Err(PlanError::WrongDof { .. })
        ));
    }

    #[test]
    fn unreachable_goal_exhausts_iterations() {
        // Goal enclosed by a ring the planner cannot cross.
        let ring_invalid = |q: &[f64]| {
            let r = (q[0] * q[0] + q[1] * q[1]).sqrt();
            !(0.3..=0.5).contains(&r)
        };
        let mut is_valid = ring_invalid;
        let options = PlanOptions {
            max_iters: 300,
            ..PlanOptions::default()
        };
        assert!(matches!(
            plan(
                &space2(),
                &[-0.9, 0.0],
                &[0.0, 0.0],
                &mut is_valid,
                &options
            ),
            Err(PlanError::NotFound { .. })
        ));
    }
}
