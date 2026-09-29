//! Garment cloth for botrail cells.
//!
//! A [`ClothCell`] owns one garment simulated by `rapier-cloth` next to a
//! bake: it keeps a small Rapier world of its own with the table and one
//! kinematic body per gripper, takes the grippers' poses and grasp events per
//! sample, steps the cloth transactionally, and reads the result back as a
//! [`ClothTrack`] of per-sample vertex positions on the bake's sample grid.
//! The host (the rollout) owns robots, IK and timing; it only has to hand
//! over end-effector poses. Design: `.internal/docs/cloth-adapter.md`.
//!
//! Every pose, position and track of this API is in botrail's Z-up world
//! frame. The cloth simulation is Y-up; the cell converts at the boundary
//! (`world = R_x(+90°) · sim`, the usual Y-up to Z-up rotation).

use ::nalgebra::Isometry3;
use rapier_cloth::core::garment::{Garment, GarmentLayers, Landmark, Side, TShirtPattern};
use rapier_cloth::rapier::prelude::*;
use rapier_cloth::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub use rapier_cloth::core::garment::{NeckShape, SeamJoin};

#[derive(Debug, thiserror::Error)]
pub enum ClothCellError {
    #[error("cloth: {0}")]
    Cloth(String),
    #[error("gripper {0} does not exist")]
    Gripper(usize),
    #[error("unknown landmark {0:?}")]
    Landmark(String),
    #[error("gripper {0} holds nothing")]
    NotHolding(usize),
    #[error("gripper {0} already holds a patch")]
    Holding(usize),
    #[error("cloth step rejected and rolled back: {0}")]
    Rejected(String),
}

/// The garment and its shell, in metres, kilograms and seconds.
#[derive(Debug, Clone)]
pub struct GarmentSpec {
    pub pattern: TShirtPattern,
    /// World position (Z-up) of the pattern centre. With a table, `z` is
    /// replaced by the resting height above it.
    pub origin: [f64; 3],
    /// Rotation of the flat garment about the vertical axis through `origin`,
    /// radians. At zero its width runs along `+x` and its length (hem to
    /// shoulders) along `-y`.
    pub yaw: f64,
    /// Contact thickness of the cloth (midsurface to midsurface).
    pub thickness: f64,
    /// Barrier band of the implicit solver; the layers start `thickness +
    /// band` apart, which is also the stitch length of stitched seams.
    pub band: f64,
    pub friction: f64,
    /// Areal density, kg/m².
    pub surface_density: f64,
    /// Solver lanes: 1 or 4.
    pub workers: usize,
    pub max_iterations: usize,
    /// Extra membrane stiffness along and across the garment's length, Pa.
    pub warp_stiffness: f64,
    pub weft_stiffness: f64,
}

impl Default for GarmentSpec {
    fn default() -> Self {
        GarmentSpec {
            pattern: TShirtPattern::default(),
            origin: [0.0; 3],
            yaw: 0.0,
            thickness: 0.000318,
            band: 0.001,
            friction: 0.5,
            surface_density: 0.1503,
            workers: 4,
            max_iterations: 128,
            warp_stiffness: 0.0,
            weft_stiffness: 0.0,
        }
    }
}

/// Which layers a gripper closes on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatchLayers {
    /// The front panel and the seams: a gripper on the top layer.
    Top,
    /// Every layer within the radius: a pinch through the garment.
    All,
}

/// A grasp: a patch around a named landmark.
#[derive(Debug, Clone)]
pub struct GraspSpec {
    /// One of [`LANDMARKS`].
    pub landmark: String,
    /// Patch radius in the pattern's rest space (m).
    pub radius: f64,
    pub layers: PatchLayers,
    /// 0 pins the patch to the gripper; a positive compliance (m/N) holds it
    /// with springs of stiffness 1/compliance, which lets a folded flap sag
    /// between two grippers without stretching the cloth.
    pub compliance: f64,
}

/// Landmark names accepted by [`GraspSpec`] and reported by [`ClothTrack`].
pub const LANDMARKS: [&str; 12] = [
    "hem_left",
    "hem_right",
    "hem_center",
    "cuff_left",
    "cuff_right",
    "shoulder_left",
    "shoulder_right",
    "underarm_left",
    "underarm_right",
    "neck_front",
    "neck_back",
    "chest",
];

fn landmark_by_name(name: &str) -> Option<Landmark> {
    Some(match name {
        "hem_left" => Landmark::HemCorner(Side::Left),
        "hem_right" => Landmark::HemCorner(Side::Right),
        "hem_center" => Landmark::HemCenter,
        "cuff_left" => Landmark::CuffCenter(Side::Left),
        "cuff_right" => Landmark::CuffCenter(Side::Right),
        "shoulder_left" => Landmark::Shoulder(Side::Left),
        "shoulder_right" => Landmark::Shoulder(Side::Right),
        "underarm_left" => Landmark::Underarm(Side::Left),
        "underarm_right" => Landmark::Underarm(Side::Right),
        "neck_front" => Landmark::NeckFront,
        "neck_back" => Landmark::NeckBack,
        "chest" => Landmark::Chest,
        _ => return None,
    })
}

/// One accepted step.
#[derive(Debug, Clone, Copy)]
pub struct StepOutcome {
    pub converged: bool,
    pub iterations: usize,
    pub contacts: usize,
    /// Largest edge length ratio minus one.
    pub max_edge_extension: f64,
}

/// Per-sample vertex positions of one garment, aligned with the bake's sample
/// grid: sample 0 is the initial state, sample `k` the state after `k` steps.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ClothTrack {
    pub name: String,
    pub triangles: Vec<[u32; 3]>,
    /// World positions (Z-up metres) per vertex per sample.
    pub points: Vec<Vec<[f32; 3]>>,
    /// Vertices held by a gripper at each sample.
    pub held: Vec<Vec<u32>>,
    pub landmarks: BTreeMap<String, u32>,
    /// Step between samples (s).
    pub h: f64,
}

/// World (Z-up) to simulation (Y-up) coordinates: `sim = R_x(-90°) · world`.
fn to_sim(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[2], -p[1])
}
/// Simulation (Y-up) to world (Z-up) coordinates: `world = R_x(+90°) · sim`.
fn to_world(p: Vec3) -> [f64; 3] {
    [p.x, -p.z, p.y]
}
/// A world pose as the simulation sees it: the same rigid motion composed
/// with the frame change, so local anchors mean the same in both frames.
fn to_pose(iso: &Isometry3<f64>) -> Pose {
    let t = iso.translation.vector;
    let q = iso.rotation.quaternion();
    let mut pose = Pose::from_translation(to_sim([t.x, t.y, t.z]));
    pose.rotation = Rotation::from_axis_angle(Vec3::X, -std::f64::consts::FRAC_PI_2)
        * Rotation::from_xyzw(q.i, q.j, q.k, q.w);
    pose
}

/// A garment on a table with kinematic grippers, stepped next to a bake.
pub struct ClothCell {
    id: WorldId,
    rigid: PhysicsWorld,
    world: RapierClothWorld,
    cloth: ClothHandle,
    garment: Garment,
    grippers: Vec<RigidBodyHandle>,
    held: Vec<Option<(AttachmentHandle, Vec<u32>)>>,
    step: u64,
    h: f64,
    samples: Vec<Vec<[f32; 3]>>,
    held_samples: Vec<Vec<u32>>,
}

impl ClothCell {
    /// Builds the garment lying flat, front up, its lowest layer resting on
    /// the table whose top is at height `table_top` (or at `spec.origin[2]`
    /// without a table), and `grippers` kinematic bodies without colliders
    /// parked at the origin. The garment's width runs along `+x` and its
    /// length from the hem to the shoulders along `-y`, turned by `spec.yaw`.
    pub fn new(
        spec: &GarmentSpec,
        table_top: Option<f64>,
        grippers: usize,
    ) -> Result<Self, ClothCellError> {
        let cloth_err = |e: ClothError| ClothCellError::Cloth(e.to_string());
        let garment = spec.pattern.build().map_err(cloth_err)?;
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        rigid.gravity = Vec3::new(0.0, -9.81, 0.0);
        let gap = spec.thickness + spec.band;
        let mut origin = to_sim(spec.origin);
        if let Some(top) = table_top {
            rigid.colliders.insert(
                ColliderBuilder::new(SharedShape::halfspace(Vec3::Y))
                    .translation(Vec3::Y * top)
                    .friction(spec.friction),
            );
            // Lowest layer half a thickness plus one band above the table.
            origin.y = top + 0.5 * spec.thickness + spec.band;
        }
        if spec.pattern.layers == GarmentLayers::Sewn {
            origin.y += 0.5 * gap;
        }
        let mut world = RapierClothWorld::new(id);
        world.solver_settings.max_contacts = 1 << 20;
        let mut cloth = Cloth::new(
            garment.mesh().clone(),
            ClothMaterial {
                surface_density: spec.surface_density,
                damping: 0.0,
                friction: spec.friction,
                contact_radius: spec.thickness * 0.5,
                ..Default::default()
            },
        )
        .map_err(cloth_err)?;
        let turn = Rotation::from_axis_angle(Vec3::Y, spec.yaw);
        let placed: Vec<Vec3> = garment
            .placed_positions(origin, gap)
            .map_err(cloth_err)?
            .iter()
            .map(|&p| origin + turn * (p - origin))
            .collect();
        cloth.set_positions(&placed).map_err(cloth_err)?;
        cloth
            .set_contact_settings(Some(ClothContactSettings {
                thickness: spec.thickness,
                activation_margin: spec.band,
                static_friction: spec.friction,
                kinetic_friction: spec.friction,
                self_collision: true,
                continuous_self_collision: true,
                rigid_surface_collision: true,
                continuous_rigid_collision: true,
                limits: CollisionLimits {
                    candidate_pairs: 400_000_000,
                    ccd_checks: 400_000_000,
                    retained_contacts: 1 << 20,
                },
            }))
            .map_err(cloth_err)?;
        cloth
            .set_implicit_solver(Some(ImplicitSettings {
                execution: if spec.workers >= 4 {
                    ImplicitExecution::Parallel4
                } else {
                    ImplicitExecution::Serial
                },
                max_iterations: spec.max_iterations,
                material: ShellMaterial {
                    warp_stiffness: spec.warp_stiffness,
                    weft_stiffness: spec.weft_stiffness,
                    ..Default::default()
                },
                ..Default::default()
            }))
            .map_err(cloth_err)?;
        let cloth = world.add_cloth(cloth);
        let grippers = (0..grippers)
            .map(|_| {
                rigid
                    .bodies
                    .insert(RigidBodyBuilder::kinematic_position_based().pose(Pose::IDENTITY))
            })
            .collect::<Vec<_>>();
        let held = vec![None; grippers.len()];
        let mut cell = ClothCell {
            id,
            rigid,
            world,
            cloth,
            garment,
            grippers,
            held,
            step: 0,
            h: 0.0,
            samples: Vec::new(),
            held_samples: Vec::new(),
        };
        cell.record();
        Ok(cell)
    }

    pub fn garment(&self) -> &Garment {
        &self.garment
    }
    fn sim_positions(&self) -> &[Vec3] {
        self.world
            .cloth(self.cloth)
            .expect("cell cloth")
            .positions()
    }
    /// Current world positions of every vertex.
    pub fn positions(&self) -> Vec<[f64; 3]> {
        self.sim_positions().iter().map(|&p| to_world(p)).collect()
    }
    pub fn steps(&self) -> u64 {
        self.step
    }
    /// The vertex of a named landmark, if the pattern has it.
    pub fn landmark(&self, name: &str) -> Option<u32> {
        landmark_by_name(name).and_then(|l| self.garment.landmark(l))
    }
    /// Current world position of a named landmark.
    pub fn landmark_position(&self, name: &str) -> Option<[f64; 3]> {
        self.landmark(name)
            .map(|v| to_world(self.sim_positions()[v as usize]))
    }

    fn gripper(&self, gripper: usize) -> Result<RigidBodyHandle, ClothCellError> {
        self.grippers
            .get(gripper)
            .copied()
            .ok_or(ClothCellError::Gripper(gripper))
    }
    /// Teleports a gripper (before a grasp; not during a hold).
    pub fn place_gripper(
        &mut self,
        gripper: usize,
        pose: &Isometry3<f64>,
    ) -> Result<(), ClothCellError> {
        let body = self.gripper(gripper)?;
        let pose = to_pose(pose);
        self.rigid.bodies[body].set_position(pose, true);
        self.rigid.bodies[body].set_next_kinematic_position(pose);
        Ok(())
    }
    /// The gripper's pose at the end of the next step; the cloth it holds
    /// follows, and the continuous checks see the motion in between.
    pub fn set_gripper_pose(
        &mut self,
        gripper: usize,
        pose: &Isometry3<f64>,
    ) -> Result<(), ClothCellError> {
        let body = self.gripper(gripper)?;
        self.rigid.bodies[body].set_next_kinematic_position(to_pose(pose));
        Ok(())
    }

    /// Closes a gripper on the patch around a landmark where the cloth lies
    /// now; returns the number of vertices held.
    pub fn grasp(&mut self, gripper: usize, spec: &GraspSpec) -> Result<usize, ClothCellError> {
        let body = self.gripper(gripper)?;
        if self.held[gripper].is_some() {
            return Err(ClothCellError::Holding(gripper));
        }
        let landmark = landmark_by_name(&spec.landmark)
            .ok_or_else(|| ClothCellError::Landmark(spec.landmark.clone()))?;
        let radius = spec.radius.max(1.5 * self.garment.pattern().spacing);
        let vertices = match spec.layers {
            PatchLayers::Top => self.garment.top_patch(landmark, radius),
            PatchLayers::All => self.garment.patch(landmark, radius),
        };
        if vertices.is_empty() {
            return Err(ClothCellError::Landmark(spec.landmark.clone()));
        }
        let pose = *self.rigid.bodies[body].position();
        let positions = self.sim_positions();
        let points = vertices
            .iter()
            .map(|&particle| AttachmentPoint {
                particle,
                local_anchor: pose.inverse_transform_point(positions[particle as usize]),
            })
            .collect();
        let handle = self
            .world
            .attach(
                AttachmentDesc {
                    cloth: self.cloth,
                    body,
                    points,
                    compliance: spec.compliance,
                    excluded_colliders: vec![],
                },
                &self.rigid.bodies,
                &self.rigid.colliders,
            )
            .map_err(|e| ClothCellError::Cloth(e.to_string()))?;
        let count = vertices.len();
        self.held[gripper] = Some((handle, vertices));
        Ok(count)
    }
    /// Changes how firmly a gripper holds its patch without moving it: the
    /// anchors stay and the hold becomes springs of stiffness `1 / compliance`
    /// from the next step (0 pins again). Raising the compliance tenfold per
    /// step for a few steps before [`ClothCell::release`] is a soft release: a
    /// taut flap relaxes instead of snapping free.
    pub fn soften_grasp(&mut self, gripper: usize, compliance: f64) -> Result<(), ClothCellError> {
        self.gripper(gripper)?;
        let (handle, _) = self.held[gripper]
            .as_ref()
            .ok_or(ClothCellError::NotHolding(gripper))?;
        self.world
            .set_attachment_compliance(*handle, compliance)
            .map_err(|e| ClothCellError::Cloth(e.to_string()))
    }
    /// Opens a gripper; the cloth keeps the velocity it had.
    pub fn release(&mut self, gripper: usize) -> Result<(), ClothCellError> {
        self.gripper(gripper)?;
        let (handle, _) = self.held[gripper]
            .take()
            .ok_or(ClothCellError::NotHolding(gripper))?;
        self.world
            .release(handle)
            .map_err(|e| ClothCellError::Cloth(e.to_string()))
    }
    /// Hands what `from` holds to `to` without letting go: the patch keeps its
    /// position and follows `to` from the next step.
    pub fn transfer(&mut self, from: usize, to: usize) -> Result<(), ClothCellError> {
        let body = self.gripper(to)?;
        self.gripper(from)?;
        if self.held[to].is_some() {
            return Err(ClothCellError::Holding(to));
        }
        let (handle, vertices) = self.held[from]
            .take()
            .ok_or(ClothCellError::NotHolding(from))?;
        self.world
            .transfer_attachment(
                handle,
                body,
                vec![],
                &self.rigid.bodies,
                &self.rigid.colliders,
            )
            .map_err(|e| ClothCellError::Cloth(e.to_string()))?;
        self.held[to] = Some((handle, vertices));
        Ok(())
    }

    /// Advances the cloth by `h` seconds (0.1 s is the qualified step of the
    /// implicit solver). A rejected step restores the cloth, the grippers and
    /// the grasps, and returns `Rejected`; the host decides whether to retry
    /// with a smaller motion.
    pub fn step(&mut self, h: f64) -> Result<StepOutcome, ClothCellError> {
        self.world.solver_settings.max_substep = h;
        self.rigid.integration_parameters.dt = h;
        let checkpoint = self
            .world
            .checkpoint()
            .map_err(|e| ClothCellError::Cloth(e.to_string()))?;
        let bodies_before = self.rigid.bodies.clone();
        let before = SceneSnapshot::capture(
            self.id,
            self.step,
            &self.rigid.bodies,
            &self.rigid.colliders,
        );
        self.rigid.step();
        let query = self.rigid.broad_phase.as_query_pipeline(
            self.rigid.narrow_phase.query_dispatcher(),
            &self.rigid.bodies,
            &self.rigid.colliders,
            QueryFilter::default(),
        );
        let scene = RapierScene::new(query, &before, h, self.rigid.gravity);
        match self.world.step_substep(h, &scene) {
            Ok(report) => {
                self.step += 1;
                self.h = h;
                let cloth_report = &report.cloths[0].1;
                let outcome = StepOutcome {
                    converged: cloth_report.implicit.is_some_and(|o| o.converged),
                    iterations: cloth_report.iterations,
                    contacts: cloth_report.contacts,
                    max_edge_extension: (cloth_report.max_stretch - 1.0).max(0.0),
                };
                self.record();
                Ok(outcome)
            }
            Err(error) => {
                self.world
                    .restore(&checkpoint)
                    .map_err(|e| ClothCellError::Cloth(e.to_string()))?;
                // The mirror world holds only kinematic grippers and the
                // table: their poses are the whole state worth restoring.
                self.rigid.bodies = bodies_before;
                Err(ClothCellError::Rejected(format!("{error:?}")))
            }
        }
    }

    fn record(&mut self) {
        self.samples.push(
            self.sim_positions()
                .iter()
                .map(|&p| to_world(p).map(|c| c as f32))
                .collect(),
        );
        let mut held: Vec<u32> = self
            .held
            .iter()
            .flatten()
            .flat_map(|(_, v)| v.iter().copied())
            .collect();
        held.sort_unstable();
        self.held_samples.push(held);
    }

    /// The samples so far as a track.
    pub fn track(&self, name: &str) -> ClothTrack {
        let mut landmarks = BTreeMap::new();
        for name in LANDMARKS {
            if let Some(v) = self.landmark(name) {
                landmarks.insert(name.to_string(), v);
            }
        }
        ClothTrack {
            name: name.to_string(),
            triangles: self.garment.mesh().triangles().to_vec(),
            points: self.samples.clone(),
            held: self.held_samples.clone(),
            landmarks,
            h: self.h,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ::nalgebra::{Translation3, UnitQuaternion, Vector3};

    fn coarse() -> GarmentSpec {
        GarmentSpec {
            pattern: TShirtPattern {
                spacing: 0.05,
                ..TShirtPattern::default()
            },
            workers: 1,
            ..GarmentSpec::default()
        }
    }

    #[test]
    fn a_gripper_lifts_the_cuff_and_hands_it_over() {
        let mut cell = ClothCell::new(&coarse(), Some(0.0), 2).unwrap();
        let cuff = cell.landmark_position("cuff_left").unwrap();
        assert!(cuff[2] > 0.0 && cuff[2] < 0.01, "{cuff:?}");
        cell.place_gripper(0, &Isometry3::translation(cuff[0], cuff[1], cuff[2]))
            .unwrap();
        cell.place_gripper(1, &Isometry3::translation(1.0, 0.5, 0.0))
            .unwrap();
        for _ in 0..3 {
            let outcome = cell.step(0.1).unwrap();
            assert!(outcome.converged);
        }
        let held = cell
            .grasp(
                0,
                &GraspSpec {
                    landmark: "cuff_left".into(),
                    radius: 0.06,
                    layers: PatchLayers::All,
                    compliance: 0.0,
                },
            )
            .unwrap();
        assert!(held >= 2, "{held}");
        assert!(matches!(
            cell.grasp(
                0,
                &GraspSpec {
                    landmark: "chest".into(),
                    radius: 0.04,
                    layers: PatchLayers::Top,
                    compliance: 0.0
                }
            ),
            Err(ClothCellError::Holding(0))
        ));
        let cuff_vertex = cell.landmark("cuff_left").unwrap() as usize;
        let start = cell.positions()[cuff_vertex];
        for k in 1..=5 {
            cell.set_gripper_pose(
                0,
                &Isometry3::translation(cuff[0], cuff[1], cuff[2] + 0.01 * k as f64),
            )
            .unwrap();
            cell.step(0.1).unwrap();
        }
        let lifted = cell.positions()[cuff_vertex];
        assert!(
            (lifted[2] - start[2] - 0.05).abs() < 1e-6
                && (lifted[0] - start[0]).abs() < 1e-6
                && (lifted[1] - start[1]).abs() < 1e-6,
            "{start:?} -> {lifted:?}"
        );
        // Hand over to the second gripper where the first one is.
        let pose = Isometry3::translation(cuff[0], cuff[1], cuff[2] + 0.05);
        cell.place_gripper(1, &pose).unwrap();
        cell.transfer(0, 1).unwrap();
        assert!(matches!(
            cell.release(0),
            Err(ClothCellError::NotHolding(0))
        ));
        cell.set_gripper_pose(1, &Isometry3::translation(cuff[0], cuff[1], cuff[2] + 0.06))
            .unwrap();
        cell.set_gripper_pose(0, &Isometry3::translation(0.5, 0.0, 0.5))
            .unwrap();
        cell.step(0.1).unwrap();
        let moved = cell.positions()[cuff_vertex];
        assert!(
            (moved[2] - lifted[2] - 0.01).abs() < 1e-6,
            "{lifted:?} -> {moved:?}"
        );
        cell.release(1).unwrap();
        cell.step(0.1).unwrap();
        // The track covers the initial state and every accepted step.
        let track = cell.track("shirt");
        assert_eq!(track.points.len(), cell.steps() as usize + 1);
        assert_eq!(track.points[0].len(), cell.positions().len());
        let last = track.points.last().unwrap()[cuff_vertex];
        let now = cell.positions()[cuff_vertex];
        assert!((0..3).all(|i| (last[i] as f64 - now[i]).abs() < 1e-6));
        assert_eq!(track.h, 0.1);
        assert!(track.held[4].len() >= 2 && track.held.last().unwrap().is_empty());
        assert_eq!(track.landmarks["cuff_left"], cuff_vertex as u32);
        let json = serde_json::to_string(&track).unwrap();
        let back: ClothTrack = serde_json::from_str(&json).unwrap();
        assert_eq!(back, track);
    }

    #[test]
    fn unknown_landmarks_and_grippers_are_errors() {
        let mut cell = ClothCell::new(&coarse(), Some(0.0), 1).unwrap();
        assert!(matches!(
            cell.place_gripper(3, &Isometry3::identity()),
            Err(ClothCellError::Gripper(3))
        ));
        assert!(matches!(
            cell.grasp(
                0,
                &GraspSpec {
                    landmark: "pocket".into(),
                    radius: 0.05,
                    layers: PatchLayers::All,
                    compliance: 0.0
                }
            ),
            Err(ClothCellError::Landmark(_))
        ));
        assert!(cell.landmark("neck_back").is_some());
        assert_eq!(LANDMARKS.len(), 12);
    }

    #[test]
    fn the_cell_is_z_up_and_turns_the_garment_by_yaw() {
        let cell = ClothCell::new(&coarse(), Some(0.7), 0).unwrap();
        let hem = cell.landmark_position("hem_center").unwrap();
        let neck = cell.landmark_position("neck_back").unwrap();
        let left = cell.landmark_position("hem_left").unwrap();
        let right = cell.landmark_position("hem_right").unwrap();
        // Every vertex rests just above the table top.
        for p in cell.positions() {
            assert!(p[2] > 0.7 && p[2] < 0.71, "{p:?}");
        }
        // Width along x about the origin, length along -y.
        assert!((left[0] + right[0]).abs() < 1e-9 && (left[0] - right[0]).abs() > 0.4);
        assert!(
            hem[1] - neck[1] > 0.5 && (hem[0] - neck[0]).abs() < 1e-9,
            "{hem:?} {neck:?}"
        );
        // A quarter turn sends the length along +x.
        let turned = ClothCell::new(
            &GarmentSpec {
                yaw: std::f64::consts::FRAC_PI_2,
                origin: [1.0, 2.0, 0.0],
                ..coarse()
            },
            Some(0.7),
            0,
        )
        .unwrap();
        let hem = turned.landmark_position("hem_center").unwrap();
        let neck = turned.landmark_position("neck_back").unwrap();
        assert!(
            neck[0] - hem[0] > 0.5 && (neck[1] - hem[1]).abs() < 1e-9,
            "{hem:?} {neck:?}"
        );
        assert!((hem[1] - 2.0).abs() < 1e-9 && hem[2] > 0.7, "{hem:?}");
    }

    #[test]
    fn gripper_rotations_turn_the_held_patch_about_the_gripper() {
        let mut cell = ClothCell::new(&coarse(), Some(0.0), 1).unwrap();
        let cuff = cell.landmark_position("cuff_right").unwrap();
        let origin = Vector3::new(cuff[0], cuff[1], cuff[2]);
        cell.place_gripper(0, &Isometry3::translation(cuff[0], cuff[1], cuff[2]))
            .unwrap();
        let held = cell
            .grasp(
                0,
                &GraspSpec {
                    landmark: "cuff_right".into(),
                    radius: 0.08,
                    layers: PatchLayers::All,
                    compliance: 0.0,
                },
            )
            .unwrap();
        assert!(held >= 3, "{held}");
        // Lift straight up, then turn about the vertical axis through the
        // gripper: the patch turns rigidly with it, in world coordinates.
        for k in 1..=8 {
            cell.set_gripper_pose(
                0,
                &Isometry3::translation(cuff[0], cuff[1], cuff[2] + 0.01 * k as f64),
            )
            .unwrap();
            cell.step(0.1).unwrap();
        }
        let vertices = cell.held[0].as_ref().unwrap().1.clone();
        let before = cell.positions();
        let centre = origin + Vector3::new(0.0, 0.0, 0.08);
        let turn = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        cell.set_gripper_pose(0, &Isometry3::from_parts(Translation3::from(centre), turn))
            .unwrap();
        cell.step(0.1).unwrap();
        let after = cell.positions();
        for &v in &vertices {
            let p = Vector3::from(before[v as usize]);
            let q = Vector3::from(after[v as usize]);
            let expected = centre + turn * (p - centre);
            assert!(
                (q - expected).norm() < 1e-6,
                "{p:?} -> {q:?} vs {expected:?}"
            );
        }
    }

    #[test]
    fn a_softened_grasp_holds_the_patch_on_springs() {
        let mut cell = ClothCell::new(&coarse(), Some(0.0), 1).unwrap();
        assert!(matches!(
            cell.soften_grasp(0, 0.1),
            Err(ClothCellError::NotHolding(0))
        ));
        let cuff = cell.landmark_position("cuff_left").unwrap();
        cell.place_gripper(0, &Isometry3::translation(cuff[0], cuff[1], cuff[2]))
            .unwrap();
        cell.grasp(
            0,
            &GraspSpec {
                landmark: "cuff_left".into(),
                radius: 0.06,
                layers: PatchLayers::All,
                compliance: 0.0,
            },
        )
        .unwrap();
        for k in 1..=5 {
            cell.set_gripper_pose(
                0,
                &Isometry3::translation(cuff[0], cuff[1], cuff[2] + 0.01 * k as f64),
            )
            .unwrap();
            cell.step(0.1).unwrap();
        }
        let cuff_vertex = cell.landmark("cuff_left").unwrap() as usize;
        let pinned = cell.positions()[cuff_vertex];
        assert!(matches!(
            cell.soften_grasp(0, -1.0),
            Err(ClothCellError::Cloth(_))
        ));
        cell.soften_grasp(0, 0.2).unwrap();
        for _ in 0..5 {
            cell.step(0.1).unwrap();
        }
        let sagged = cell.positions()[cuff_vertex];
        let sag = pinned[2] - sagged[2];
        assert!(sag > 1e-4 && sag < 0.05, "{pinned:?} -> {sagged:?}");
        assert!(!cell.track("shirt").held.last().unwrap().is_empty());
        cell.release(0).unwrap();
        assert!(matches!(
            cell.soften_grasp(0, 0.0),
            Err(ClothCellError::NotHolding(0))
        ));
    }
}
