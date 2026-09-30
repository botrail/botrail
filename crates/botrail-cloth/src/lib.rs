//! Cloth for botrail cells.
//!
//! A [`ClothCell`] owns one cloth — a parametric T-shirt or a plain sheet —
//! simulated by `rapier-cloth` next to a bake: a small Rapier world of its
//! own with the support surface and one kinematic body per gripper. It takes
//! the grippers' poses and grasp events and steps the cloth transactionally.
//! [`pass::animate`] drives a cell from a baked cycle — the robots' tool
//! poses and the signals that close their grippers — and returns the
//! [`ClothTrack`] the timeline carries. The cloth never drives a robot, so
//! the pass runs after the bake. Design: `.internal/docs/cloth-adapter.md`.
//!
//! Every pose and position of this API is in botrail's Z-up world frame. The
//! cloth simulation is Y-up; the cell converts at the boundary
//! (`world = R_x(+90°) · sim`, the usual Y-up to Z-up rotation).

pub mod pass;

use ::nalgebra::Isometry3;
use rapier_cloth::core::garment::{Garment, Landmark, Side};
use rapier_cloth::rapier::prelude::*;
use rapier_cloth::*;
use std::collections::BTreeMap;

pub use botrail_scene::cloth::ClothTrack;
pub use rapier_cloth::core::garment::{GarmentLayers, NeckShape, SeamJoin, TShirtPattern};

#[derive(Debug, thiserror::Error)]
pub enum ClothCellError {
    #[error("cloth: {0}")]
    Cloth(String),
    #[error("gripper {0} does not exist")]
    Gripper(usize),
    #[error("gripper {0} holds nothing")]
    NotHolding(usize),
    #[error("gripper {0} already holds a patch")]
    Holding(usize),
    #[error("cloth step rejected and rolled back: {0}")]
    Rejected(String),
    /// A pass was asked for something the cycle or the scene does not have.
    #[error("{0}")]
    Pass(String),
}

/// What the cloth is cut as.
#[derive(Debug, Clone)]
pub enum ClothShape {
    /// A parametric T-shirt: one panel, or a front and a back sewn together.
    TShirt(TShirtPattern),
    /// A rectangular sheet `size[0]` wide and `size[1]` long, on square
    /// cells of about `spacing`.
    Sheet { size: [f64; 2], spacing: f64 },
}

/// The cloth and its shell, in metres, kilograms and seconds.
#[derive(Debug, Clone)]
pub struct ClothSpec {
    pub shape: ClothShape,
    /// World position (Z-up) of the cloth's centre. With a support surface,
    /// `z` is replaced by the resting height above it.
    pub origin: [f64; 3],
    /// Rotation of the flat cloth about the vertical axis through `origin`,
    /// radians. At zero its width runs along `+x` and its length (a
    /// T-shirt's hem to its shoulders) along `-y`.
    pub yaw: f64,
    /// Contact thickness of the cloth (midsurface to midsurface).
    pub thickness: f64,
    /// Barrier band of the implicit solver; a sewn garment's layers start
    /// `thickness + band` apart, which is also the stitch length of
    /// stitched seams.
    pub band: f64,
    pub friction: f64,
    /// Areal density, kg/m².
    pub surface_density: f64,
    /// Solver lanes: 1 or 4.
    pub workers: usize,
    pub max_iterations: usize,
    /// Extra membrane stiffness along and across a garment's length, Pa.
    pub warp_stiffness: f64,
    pub weft_stiffness: f64,
}

impl Default for ClothSpec {
    fn default() -> Self {
        ClothSpec {
            shape: ClothShape::TShirt(TShirtPattern::default()),
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

/// A box a gripper carries into the cloth world, so that it pushes cloth it
/// does not hold. Half extents and offset are in the gripper's own (tool)
/// frame. The cloth a gripper holds does not collide with its own pad.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pad {
    pub half_extents: [f64; 3],
    /// Centre of the box in the tool frame.
    pub offset: [f64; 3],
}

/// Landmark names of a T-shirt.
pub const TSHIRT_LANDMARKS: [&str; 12] = [
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

/// Landmark names of a sheet, as it lies at yaw 0: north is `+y`, east `+x`.
pub const SHEET_LANDMARKS: [&str; 9] = [
    "corner_nw",
    "corner_ne",
    "corner_sw",
    "corner_se",
    "edge_n",
    "edge_s",
    "edge_w",
    "edge_e",
    "center",
];

fn tshirt_landmark(name: &str) -> Option<Landmark> {
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

/// The cloth as it lies before the first step.
struct Layout {
    mesh: ClothMesh,
    /// Starting positions (simulation frame).
    placed: Vec<Vec3>,
    landmarks: BTreeMap<String, u32>,
    garment: Option<Garment>,
}

fn layout(spec: &ClothSpec, table_top: Option<f64>) -> Result<Layout, ClothCellError> {
    let cloth_err = |e: ClothError| ClothCellError::Cloth(e.to_string());
    let gap = spec.thickness + spec.band;
    let mut origin = to_sim(spec.origin);
    if let Some(top) = table_top {
        // Lowest layer half a thickness plus one band above the surface.
        origin.y = top + 0.5 * spec.thickness + spec.band;
    }
    let turn = Rotation::from_axis_angle(Vec3::Y, spec.yaw);
    match &spec.shape {
        ClothShape::TShirt(pattern) => {
            let garment = pattern.build().map_err(cloth_err)?;
            if pattern.layers == GarmentLayers::Sewn {
                origin.y += 0.5 * gap;
            }
            let placed = garment
                .placed_positions(origin, gap)
                .map_err(cloth_err)?
                .iter()
                .map(|&p| origin + turn * (p - origin))
                .collect();
            let landmarks = TSHIRT_LANDMARKS
                .iter()
                .filter_map(|name| {
                    let vertex = garment.landmark(tshirt_landmark(name)?)?;
                    Some((name.to_string(), vertex))
                })
                .collect();
            Ok(Layout {
                mesh: garment.mesh().clone(),
                placed,
                landmarks,
                garment: Some(garment),
            })
        }
        ClothShape::Sheet { size, spacing } => {
            let [width, length] = *size;
            let positive = |v: f64| v.is_finite() && v > 0.0;
            if !(positive(*spacing) && positive(width) && positive(length)) {
                return Err(ClothCellError::Cloth(
                    "a sheet needs a positive size and spacing".into(),
                ));
            }
            // Vertices per side; at least one cell each way.
            let nx = (width / spacing).round().max(1.0) as usize + 1;
            let ny = (length / spacing).round().max(1.0) as usize + 1;
            // Width along x, length along the simulation's z (world -y),
            // centred on the origin.
            let mesh = GridBuilder::new(nx, ny)
                .size(width, length)
                .origin(Vec3::new(-0.5 * width, 0.0, -0.5 * length))
                .build()
                .map_err(cloth_err)?;
            let placed = mesh
                .rest_positions()
                .iter()
                .map(|&p| origin + turn * p)
                .collect();
            let at = |x: usize, y: usize| (y * nx + x) as u32;
            let (ex, ey, mx, my) = (nx - 1, ny - 1, (nx - 1) / 2, (ny - 1) / 2);
            let landmarks = [
                ("corner_nw", at(0, 0)),
                ("corner_ne", at(ex, 0)),
                ("corner_sw", at(0, ey)),
                ("corner_se", at(ex, ey)),
                ("edge_n", at(mx, 0)),
                ("edge_s", at(mx, ey)),
                ("edge_w", at(0, my)),
                ("edge_e", at(ex, my)),
                ("center", at(mx, my)),
            ]
            .into_iter()
            .map(|(name, vertex)| (name.to_string(), vertex))
            .collect();
            Ok(Layout {
                mesh,
                placed,
                landmarks,
                garment: None,
            })
        }
    }
}

/// Where the cloth's landmarks lie before the first step (world, Z-up) —
/// what a cell's motions are taught against. Builds the cloth's layout
/// only; nothing is simulated.
pub fn rest_landmarks(
    spec: &ClothSpec,
    table_top: Option<f64>,
) -> Result<BTreeMap<String, [f64; 3]>, ClothCellError> {
    let layout = layout(spec, table_top)?;
    Ok(layout
        .landmarks
        .iter()
        .map(|(name, &vertex)| (name.clone(), to_world(layout.placed[vertex as usize])))
        .collect())
}

/// The rigid world as it was before a step, to put back when the cloth
/// rejects it.
struct RigidCheckpoint {
    gravity: Vec3,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
}

impl RigidCheckpoint {
    fn capture(world: &PhysicsWorld) -> Self {
        Self {
            gravity: world.gravity,
            integration_parameters: world.integration_parameters,
            islands: world.islands.clone(),
            broad_phase: world.broad_phase.clone(),
            narrow_phase: world.narrow_phase.clone(),
            bodies: world.bodies.clone(),
            colliders: world.colliders.clone(),
            impulse_joints: world.impulse_joints.clone(),
            multibody_joints: world.multibody_joints.clone(),
            ccd_solver: world.ccd_solver.clone(),
        }
    }
    fn restore(self, world: &mut PhysicsWorld) {
        *world = PhysicsWorld {
            gravity: self.gravity,
            integration_parameters: self.integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            islands: self.islands,
            broad_phase: self.broad_phase,
            narrow_phase: self.narrow_phase,
            bodies: self.bodies,
            colliders: self.colliders,
            impulse_joints: self.impulse_joints,
            multibody_joints: self.multibody_joints,
            ccd_solver: self.ccd_solver,
        };
    }
}

/// A cloth on a support surface with kinematic grippers, stepped next to a
/// bake.
pub struct ClothCell {
    id: WorldId,
    rigid: PhysicsWorld,
    world: RapierClothWorld,
    cloth: ClothHandle,
    triangles: Vec<[u32; 3]>,
    landmarks: BTreeMap<String, u32>,
    garment: Option<Garment>,
    grippers: Vec<RigidBodyHandle>,
    /// The pad collider of each gripper that has one.
    pads: Vec<Option<ColliderHandle>>,
    held: Vec<Option<(AttachmentHandle, Vec<u32>)>>,
    step: u64,
}

impl ClothCell {
    /// Builds the cloth lying flat (a T-shirt front up), its lowest layer
    /// resting on the support surface at height `table_top` (or at
    /// `spec.origin[2]` without one), and one kinematic body per entry of
    /// `grippers`, parked at the origin — with the entry's pad as its
    /// collider, if any. The support is a level plane: cloth does not fall
    /// off a table's edge.
    pub fn new(
        spec: &ClothSpec,
        table_top: Option<f64>,
        grippers: &[Option<Pad>],
    ) -> Result<Self, ClothCellError> {
        let cloth_err = |e: ClothError| ClothCellError::Cloth(e.to_string());
        let Layout {
            mesh,
            placed,
            landmarks,
            garment,
        } = layout(spec, table_top)?;
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        rigid.gravity = Vec3::new(0.0, -9.81, 0.0);
        if let Some(top) = table_top {
            rigid.colliders.insert(
                ColliderBuilder::new(SharedShape::halfspace(Vec3::Y))
                    .translation(Vec3::Y * top)
                    .friction(spec.friction),
            );
        }
        let mut world = RapierClothWorld::new(id);
        world.solver_settings.max_contacts = 1 << 20;
        let triangles = mesh.triangles().to_vec();
        let mut cloth = Cloth::new(
            mesh,
            ClothMaterial {
                surface_density: spec.surface_density,
                damping: 0.0,
                friction: spec.friction,
                contact_radius: spec.thickness * 0.5,
                ..Default::default()
            },
        )
        .map_err(cloth_err)?;
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
        let mut bodies = Vec::with_capacity(grippers.len());
        let mut pads = Vec::with_capacity(grippers.len());
        for pad in grippers {
            let body = rigid
                .bodies
                .insert(RigidBodyBuilder::kinematic_position_based().pose(Pose::IDENTITY));
            pads.push(match pad {
                Some(pad) => {
                    let [hx, hy, hz] = pad.half_extents;
                    if !(hx > 0.0 && hy > 0.0 && hz > 0.0) {
                        return Err(ClothCellError::Cloth(
                            "a pad needs positive half extents".into(),
                        ));
                    }
                    let [ox, oy, oz] = pad.offset;
                    // The body's frame is the tool frame, so the pad's
                    // local placement needs no frame change.
                    Some(
                        rigid.colliders.insert_with_parent(
                            ColliderBuilder::cuboid(hx, hy, hz)
                                .translation(Vec3::new(ox, oy, oz))
                                .friction(spec.friction),
                            body,
                            &mut rigid.bodies,
                        ),
                    )
                }
                None => None,
            });
            bodies.push(body);
        }
        let held = vec![None; bodies.len()];
        Ok(ClothCell {
            id,
            rigid,
            world,
            cloth,
            triangles,
            landmarks,
            garment,
            grippers: bodies,
            pads,
            held,
            step: 0,
        })
    }

    /// The garment's pattern queries (panels, regions, patches); `None`
    /// for a sheet.
    pub fn garment(&self) -> Option<&Garment> {
        self.garment.as_ref()
    }
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }
    /// Landmark name to vertex ([`TSHIRT_LANDMARKS`], [`SHEET_LANDMARKS`]).
    pub fn landmarks(&self) -> &BTreeMap<String, u32> {
        &self.landmarks
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
    /// Accepted steps so far.
    pub fn steps(&self) -> u64 {
        self.step
    }
    /// The vertex of a named landmark, if the cloth has it.
    pub fn landmark(&self, name: &str) -> Option<u32> {
        self.landmarks.get(name).copied()
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

    /// The vertices a gripper holds, if it holds any.
    pub fn held(&self, gripper: usize) -> Option<&[u32]> {
        self.held
            .get(gripper)?
            .as_ref()
            .map(|(_, vertices)| vertices.as_slice())
    }
    /// The gripper holding `vertex`, if one does.
    pub fn holder_of(&self, vertex: u32) -> Option<usize> {
        self.held.iter().position(|held| {
            held.as_ref()
                .is_some_and(|(_, vertices)| vertices.contains(&vertex))
        })
    }
    /// Every held vertex, ascending.
    pub fn held_vertices(&self) -> Vec<u32> {
        let mut held: Vec<u32> = self
            .held
            .iter()
            .flatten()
            .flat_map(|(_, vertices)| vertices.iter().copied())
            .collect();
        held.sort_unstable();
        held
    }
    /// The vertices within `radius` of a gripper's origin where the cloth
    /// lies now, through every layer.
    pub fn near(&self, gripper: usize, radius: f64) -> Result<Vec<u32>, ClothCellError> {
        let body = self.gripper(gripper)?;
        let centre = self.rigid.bodies[body].position().translation;
        Ok(self
            .sim_positions()
            .iter()
            .enumerate()
            .filter(|(_, p)| p.distance(centre) <= radius)
            .map(|(i, _)| i as u32)
            .collect())
    }

    /// Closes a gripper on the free vertices within `radius` of its origin
    /// (a pinch through every layer); returns how many it holds. Zero
    /// means it closed on nothing and holds nothing. `compliance` 0 pins
    /// the patch to the gripper; a positive compliance (m/N) holds it with
    /// springs of stiffness `1 / compliance`, which lets a folded flap sag
    /// between two grippers without stretching the cloth.
    pub fn grasp_near(
        &mut self,
        gripper: usize,
        radius: f64,
        compliance: f64,
    ) -> Result<usize, ClothCellError> {
        let vertices: Vec<u32> = self
            .near(gripper, radius)?
            .into_iter()
            .filter(|&v| self.holder_of(v).is_none())
            .collect();
        if self.held[gripper].is_some() {
            return Err(ClothCellError::Holding(gripper));
        }
        if vertices.is_empty() {
            return Ok(0);
        }
        self.grasp_vertices(gripper, vertices, compliance)
    }

    /// Closes a gripper on chosen vertices where they are now (a garment's
    /// top-layer patch, say — see [`ClothCell::garment`]); returns how many.
    pub fn grasp_vertices(
        &mut self,
        gripper: usize,
        vertices: Vec<u32>,
        compliance: f64,
    ) -> Result<usize, ClothCellError> {
        let body = self.gripper(gripper)?;
        if self.held[gripper].is_some() {
            return Err(ClothCellError::Holding(gripper));
        }
        let pose = *self.rigid.bodies[body].position();
        let positions = self.sim_positions();
        let mut points = Vec::with_capacity(vertices.len());
        for &particle in &vertices {
            let position = positions.get(particle as usize).ok_or_else(|| {
                ClothCellError::Cloth(format!("vertex {particle} does not exist"))
            })?;
            points.push(AttachmentPoint {
                particle,
                local_anchor: pose.inverse_transform_point(*position),
            });
        }
        let handle = self
            .world
            .attach(
                AttachmentDesc {
                    cloth: self.cloth,
                    body,
                    points,
                    compliance,
                    excluded_colliders: self.pads[gripper].into_iter().collect(),
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
                self.pads[to].into_iter().collect(),
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
        let rigid_before = RigidCheckpoint::capture(&self.rigid);
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
                let cloth_report = &report.cloths[0].1;
                Ok(StepOutcome {
                    converged: cloth_report.implicit.is_some_and(|o| o.converged),
                    iterations: cloth_report.iterations,
                    contacts: cloth_report.contacts,
                    max_edge_extension: (cloth_report.max_stretch - 1.0).max(0.0),
                })
            }
            Err(error) => {
                self.world
                    .restore(&checkpoint)
                    .map_err(|e| ClothCellError::Cloth(e.to_string()))?;
                rigid_before.restore(&mut self.rigid);
                Err(ClothCellError::Rejected(format!("{error:?}")))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ::nalgebra::{Translation3, UnitQuaternion, Vector3};

    fn coarse_shirt() -> ClothSpec {
        ClothSpec {
            shape: ClothShape::TShirt(TShirtPattern {
                spacing: 0.05,
                ..TShirtPattern::default()
            }),
            workers: 1,
            ..ClothSpec::default()
        }
    }

    fn sheet() -> ClothSpec {
        ClothSpec {
            shape: ClothShape::Sheet {
                size: [0.3, 0.2],
                spacing: 0.05,
            },
            workers: 1,
            ..ClothSpec::default()
        }
    }

    fn at(p: [f64; 3]) -> Isometry3<f64> {
        Isometry3::translation(p[0], p[1], p[2])
    }

    #[test]
    fn a_gripper_lifts_the_cuff_and_hands_it_over() {
        let mut cell = ClothCell::new(&coarse_shirt(), Some(0.0), &[None, None]).unwrap();
        let cuff = cell.landmark_position("cuff_left").unwrap();
        assert!(cuff[2] > 0.0 && cuff[2] < 0.01, "{cuff:?}");
        cell.place_gripper(0, &at(cuff)).unwrap();
        cell.place_gripper(1, &at([1.0, 0.0, 0.5])).unwrap();
        for _ in 0..3 {
            let outcome = cell.step(0.1).unwrap();
            assert!(outcome.converged);
        }
        // The far gripper closes on nothing; the near one pinches both layers.
        assert_eq!(cell.grasp_near(1, 0.06, 0.0).unwrap(), 0);
        assert!(cell.held(1).is_none());
        let held = cell.grasp_near(0, 0.06, 0.0).unwrap();
        assert!(held >= 2, "{held}");
        assert!(matches!(
            cell.grasp_vertices(0, vec![0], 0.0),
            Err(ClothCellError::Holding(0))
        ));
        let cuff_vertex = cell.landmark("cuff_left").unwrap() as usize;
        assert_eq!(cell.holder_of(cuff_vertex as u32), Some(0));
        let start = cell.positions()[cuff_vertex];
        for k in 1..=5 {
            cell.set_gripper_pose(0, &at([cuff[0], cuff[1], cuff[2] + 0.01 * k as f64]))
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
        cell.place_gripper(1, &at([cuff[0], cuff[1], cuff[2] + 0.05]))
            .unwrap();
        cell.transfer(0, 1).unwrap();
        assert!(matches!(
            cell.release(0),
            Err(ClothCellError::NotHolding(0))
        ));
        assert_eq!(cell.holder_of(cuff_vertex as u32), Some(1));
        cell.set_gripper_pose(1, &at([cuff[0], cuff[1], cuff[2] + 0.06]))
            .unwrap();
        cell.set_gripper_pose(0, &at([0.5, 0.0, 0.5])).unwrap();
        cell.step(0.1).unwrap();
        let moved = cell.positions()[cuff_vertex];
        assert!(
            (moved[2] - lifted[2] - 0.01).abs() < 1e-6,
            "{lifted:?} -> {moved:?}"
        );
        assert_eq!(cell.held_vertices().len(), held);
        cell.release(1).unwrap();
        cell.step(0.1).unwrap();
        assert!(cell.held_vertices().is_empty());
        assert_eq!(cell.steps(), 10);
    }

    #[test]
    fn unknown_grippers_and_vertices_are_errors() {
        let mut cell = ClothCell::new(&coarse_shirt(), Some(0.0), &[None]).unwrap();
        assert!(matches!(
            cell.place_gripper(3, &Isometry3::identity()),
            Err(ClothCellError::Gripper(3))
        ));
        assert!(matches!(
            cell.grasp_near(2, 0.05, 0.0),
            Err(ClothCellError::Gripper(2))
        ));
        assert!(matches!(
            cell.grasp_vertices(0, vec![1_000_000], 0.0),
            Err(ClothCellError::Cloth(_))
        ));
        assert!(cell.landmark("neck_back").is_some());
        assert!(cell.landmark("pocket").is_none());
        assert_eq!(cell.landmarks().len(), TSHIRT_LANDMARKS.len());
        assert!(cell.garment().is_some());
    }

    #[test]
    fn the_cell_is_z_up_and_turns_the_cloth_by_yaw() {
        let cell = ClothCell::new(&coarse_shirt(), Some(0.7), &[]).unwrap();
        let hem = cell.landmark_position("hem_center").unwrap();
        let neck = cell.landmark_position("neck_back").unwrap();
        let left = cell.landmark_position("hem_left").unwrap();
        let right = cell.landmark_position("hem_right").unwrap();
        // Every vertex rests just above the support.
        for p in cell.positions() {
            assert!(p[2] > 0.7 && p[2] < 0.71, "{p:?}");
        }
        // Width along x about the origin, length along -y.
        assert!((left[0] + right[0]).abs() < 1e-9 && (left[0] - right[0]).abs() > 0.4);
        assert!(
            hem[1] - neck[1] > 0.5 && (hem[0] - neck[0]).abs() < 1e-9,
            "{hem:?} {neck:?}"
        );
        // A quarter turn sends the length along +x; the landmarks a cell is
        // taught against are those of the cloth as it will lie.
        let turned = ClothSpec {
            yaw: std::f64::consts::FRAC_PI_2,
            origin: [1.0, 2.0, 0.0],
            ..coarse_shirt()
        };
        let rest = rest_landmarks(&turned, Some(0.7)).unwrap();
        let cell = ClothCell::new(&turned, Some(0.7), &[]).unwrap();
        let hem = cell.landmark_position("hem_center").unwrap();
        let neck = cell.landmark_position("neck_back").unwrap();
        assert!(
            neck[0] - hem[0] > 0.5 && (neck[1] - hem[1]).abs() < 1e-9,
            "{hem:?} {neck:?}"
        );
        assert!((hem[1] - 2.0).abs() < 1e-9 && hem[2] > 0.7, "{hem:?}");
        assert_eq!(rest["hem_center"], hem);
        assert_eq!(rest.len(), TSHIRT_LANDMARKS.len());
    }

    #[test]
    fn gripper_rotations_turn_the_held_patch_about_the_gripper() {
        let mut cell = ClothCell::new(&coarse_shirt(), Some(0.0), &[None]).unwrap();
        let cuff = cell.landmark_position("cuff_right").unwrap();
        let origin = Vector3::new(cuff[0], cuff[1], cuff[2]);
        cell.place_gripper(0, &at(cuff)).unwrap();
        let held = cell.grasp_near(0, 0.08, 0.0).unwrap();
        assert!(held >= 3, "{held}");
        // Lift straight up, then turn about the vertical axis through the
        // gripper: the patch turns rigidly with it, in world coordinates.
        for k in 1..=8 {
            cell.set_gripper_pose(0, &at([cuff[0], cuff[1], cuff[2] + 0.01 * k as f64]))
                .unwrap();
            cell.step(0.1).unwrap();
        }
        let vertices = cell.held(0).unwrap().to_vec();
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
        let mut cell = ClothCell::new(&coarse_shirt(), Some(0.0), &[None]).unwrap();
        assert!(matches!(
            cell.soften_grasp(0, 0.1),
            Err(ClothCellError::NotHolding(0))
        ));
        let cuff = cell.landmark_position("cuff_left").unwrap();
        cell.place_gripper(0, &at(cuff)).unwrap();
        cell.grasp_near(0, 0.06, 0.0).unwrap();
        for k in 1..=5 {
            cell.set_gripper_pose(0, &at([cuff[0], cuff[1], cuff[2] + 0.01 * k as f64]))
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
        assert!(!cell.held_vertices().is_empty());
        cell.release(0).unwrap();
        assert!(matches!(
            cell.soften_grasp(0, 0.0),
            Err(ClothCellError::NotHolding(0))
        ));
    }

    #[test]
    fn a_sheet_lies_flat_with_corner_landmarks_and_lifts_by_a_corner() {
        let spec = ClothSpec {
            origin: [0.5, 0.0, 0.0],
            ..sheet()
        };
        let mut cell = ClothCell::new(&spec, Some(0.4), &[None]).unwrap();
        assert!(cell.garment().is_none());
        assert_eq!(cell.landmarks().len(), SHEET_LANDMARKS.len());
        // 0.3 x 0.2 on 5 cm cells: 7 x 5 vertices, north at +y, east at +x.
        assert_eq!(cell.positions().len(), 35);
        assert_eq!(cell.triangles().len(), 48);
        let nw = cell.landmark_position("corner_nw").unwrap();
        let se = cell.landmark_position("corner_se").unwrap();
        let centre = cell.landmark_position("center").unwrap();
        assert!(
            (nw[0] - 0.35).abs() < 1e-9 && (nw[1] - 0.1).abs() < 1e-9,
            "{nw:?}"
        );
        assert!(
            (se[0] - 0.65).abs() < 1e-9 && (se[1] + 0.1).abs() < 1e-9,
            "{se:?}"
        );
        assert!((centre[0] - 0.5).abs() < 1e-9 && centre[1].abs() < 1e-9);
        assert!(nw[2] > 0.4 && nw[2] < 0.402, "{nw:?}");
        assert_eq!(rest_landmarks(&spec, Some(0.4)).unwrap()["corner_se"], se);
        // A pinch at the corner takes the corner cell and lifts it.
        cell.place_gripper(0, &at(se)).unwrap();
        let held = cell.grasp_near(0, 0.06, 0.0).unwrap();
        assert!((2..=4).contains(&held), "{held}");
        for k in 1..=5 {
            cell.set_gripper_pose(0, &at([se[0], se[1], se[2] + 0.02 * k as f64]))
                .unwrap();
            assert!(cell.step(0.1).unwrap().converged);
        }
        let lifted = cell.landmark_position("corner_se").unwrap();
        assert!((lifted[2] - se[2] - 0.1).abs() < 1e-6, "{lifted:?}");
        // The far corner stays on the support.
        let far = cell.landmark_position("corner_nw").unwrap();
        assert!(far[2] < 0.405, "{far:?}");
        assert!(ClothCell::new(
            &ClothSpec {
                shape: ClothShape::Sheet {
                    size: [0.3, 0.0],
                    spacing: 0.05
                },
                ..sheet()
            },
            None,
            &[]
        )
        .is_err());
    }

    #[test]
    fn a_pad_pushes_the_cloth_it_does_not_hold() {
        // A 4 cm cube standing on the support sweeps 1 cm per step along +x
        // into the west edge of a resting sheet.
        let pad = Pad {
            half_extents: [0.02, 0.02, 0.02],
            offset: [0.0, 0.0, 0.0],
        };
        let mut cell = ClothCell::new(&sheet(), Some(0.0), &[Some(pad)]).unwrap();
        let west = cell.landmark_position("edge_w").unwrap();
        let start = [west[0] - 0.03, west[1], 0.02];
        cell.place_gripper(0, &at(start)).unwrap();
        for _ in 0..2 {
            cell.step(0.1).unwrap();
        }
        let before = cell.landmark_position("edge_w").unwrap();
        for k in 1..=8 {
            cell.set_gripper_pose(0, &at([start[0] + 0.01 * k as f64, start[1], start[2]]))
                .unwrap();
            cell.step(0.1).unwrap();
        }
        // The pad's leading face ends 7 cm from where it started: 6 cm past
        // the edge's first position, which it therefore pushed ahead.
        let after = cell.landmark_position("edge_w").unwrap();
        let face = start[0] + 0.08 + 0.02;
        assert!(after[0] > before[0] + 0.04, "{before:?} -> {after:?}");
        assert!(
            after[0] > face - 0.002,
            "edge {after:?} behind the face {face}"
        );
        assert!(ClothCell::new(
            &sheet(),
            None,
            &[Some(Pad {
                half_extents: [0.02, 0.0, 0.02],
                offset: [0.0; 3]
            })]
        )
        .is_err());
    }
}
