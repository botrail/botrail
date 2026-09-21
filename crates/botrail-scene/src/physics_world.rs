//! World-scope physics (design-world-physics.md W0): what the engine owns
//! when a bake asks for the *whole cell* rather than the declared bodies.
//!
//! The declared scope hands the engine exactly what authoring marked
//! (`set_physics(dynamic=True)`). The world scope derives the rest: every
//! obstacle is folded into a *rigid unit* by its name hierarchy and part
//! identity (a pedestal's base/column/top, a pallet with its timber, a
//! workpiece with its display shell), and a unit is held fixed only when
//! authoring or identity says so — an explicit `dynamic=False`, a device
//! that moves it by name, a walkable floor, or an equipment part pin (a
//! bolted-down rack). Everything else is dynamic, a box on the floor
//! included: in a physical world an external force can move it.
//!
//! One derivation serves both the lowering (`rollout::init_physics`) and
//! the audit ([`PhysicsPlan`]) so the table a user reads is the world the
//! engine gets.

use std::collections::{BTreeMap, HashMap, HashSet};

use botrail_physics::{BodyKind, BodyProps};

use crate::dynamics::RobotDynamics;
use crate::part::{PartAttr, PartTargetKind};
use crate::rollout::{PhysicsOptions, PhysicsScope};
use crate::seq::{Action, DeviceKind, Drive};
use crate::Scene;

/// Whether robot `r` runs as a dynamic body under `options`, and with
/// what declaration: its own, or — under the world scope — the defaults
/// (the flag says whether a force cap had to be defaulted for want of an
/// effort limit). `None`: a kinematic mirror, as ever.
pub(crate) fn robot_dynamics_for(
    scene: &Scene,
    r: usize,
    options: &PhysicsOptions,
) -> Option<(RobotDynamics, bool)> {
    if let Some(declared) = scene.robot_dynamics(r) {
        return Some((declared.clone(), false));
    }
    if options.scope != PhysicsScope::World {
        return None;
    }
    crate::dynamics::resolve_dynamics(&scene.robots()[r].model, None, None, None, None, true).ok()
}

/// Whether robot `r`'s base is a free rigid body under `options`: what
/// the declaration says, else — under the world scope — a walker (a gait
/// on its mount) or an aircraft (an aerial vehicle) floats and nothing
/// else does. A bolted-down arm keeps its stand; an arm on an AGV rides
/// the (kinematic) vehicle.
pub(crate) fn robot_floating(scene: &Scene, r: usize, options: &PhysicsOptions) -> bool {
    let sr = &scene.robots()[r];
    if let Some(floating) = sr.dynamics.as_ref().and_then(|d| d.floating) {
        return floating;
    }
    if options.scope != PhysicsScope::World {
        return false;
    }
    let Some(mount) = sr.mount.as_ref() else {
        return false;
    };
    if mount.gait.is_some() {
        return true;
    }
    scene.devices().iter().any(|d| {
        d.name == mount.device
            && matches!(
                d.kind,
                DeviceKind::Vehicle {
                    drive: Drive::Aerial { .. },
                    ..
                }
            )
    })
}

/// What a planned body is to the engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanKind {
    /// The ground plane (a static half-space).
    Ground,
    /// A kinematic mirror: it never moves unless the rollout moves it.
    Fixed,
    /// The engine owns its pose.
    Dynamic,
    /// A robot the engine simulates as an articulated body (every link
    /// with geometry a body, every joint a joint); the reason says how
    /// its joints are driven and where its base is.
    Robot,
}

impl PlanKind {
    pub fn as_str(self) -> &'static str {
        match self {
            PlanKind::Ground => "ground",
            PlanKind::Fixed => "fixed",
            PlanKind::Dynamic => "dynamic",
            PlanKind::Robot => "robot",
        }
    }
}

/// One row of the audit: a rigid unit (or the ground), what it is, why,
/// and what it weighs when the engine owns it.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedBody {
    /// The unit's name: its root obstacle, or the part pin that groups it.
    pub name: String,
    pub kind: PlanKind,
    /// Every obstacle the unit carries, frame member first. Disabled
    /// members ride along without colliding.
    pub members: Vec<String>,
    /// The rule that decided `kind`.
    pub reason: String,
    /// Mass (kg) of a dynamic unit, when stated or identified; `None`
    /// derives it from the shape at the default density.
    pub mass: Option<f64>,
}

/// The audit of a physics bake's lowering: one row per unit the engine
/// sees, plus the count of obstacles mirrored individually (the declared
/// scope's default, and disabled scenery nobody lowers).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PhysicsPlan {
    pub rows: Vec<PlannedBody>,
    /// Enabled obstacles lowered as individual kinematic mirrors without
    /// a row of their own (declared scope: everything not marked).
    pub mirrors: usize,
    /// Dynamic units that start the bake interpenetrating something
    /// else — `(unit, other obstacle)` pairs, in unit order. Obstacles are
    /// never collision-checked against each other while authoring, so a
    /// part pushed into its fixture goes unnoticed until the engine
    /// ejects it; this is the warning before that happens.
    pub overlaps: Vec<(String, String)>,
}

impl PhysicsPlan {
    pub fn dynamic(&self) -> impl Iterator<Item = &PlannedBody> {
        self.rows.iter().filter(|r| r.kind == PlanKind::Dynamic)
    }

    /// The table as GitHub-flavoured markdown.
    pub fn to_markdown(&self) -> String {
        let mut out =
            String::from("| body | kind | members | mass kg | reason |\n|---|---|---|---|---|\n");
        for row in &self.rows {
            let members = if row.members.len() <= 1 {
                String::new()
            } else {
                row.members.len().to_string()
            };
            let mass = row
                .mass
                .map(|m| {
                    format!("{m:.3}")
                        .trim_end_matches('0')
                        .trim_end_matches('.')
                        .to_string()
                })
                .unwrap_or_default();
            out.push_str(&format!(
                "| {} | {} | {} | {} | {} |\n",
                row.name,
                row.kind.as_str(),
                members,
                mass,
                row.reason
            ));
        }
        if self.mirrors > 0 {
            out.push_str(&format!(
                "\n{} obstacle(s) mirrored individually (kinematic).\n",
                self.mirrors
            ));
        }
        if !self.overlaps.is_empty() {
            out.push_str(
                "\nStarting in contact (interpenetrating — the engine will push these apart):\n",
            );
            for (unit, other) in &self.overlaps {
                out.push_str(&format!("- {unit} × {other}\n"));
            }
        }
        out
    }
}

/// One rigid unit of the lowering.
#[derive(Debug, Clone)]
pub(crate) struct Unit {
    pub name: String,
    /// The obstacle the unit's frame is taken from (its root, or the first
    /// member of a pin-only group).
    pub frame: usize,
    /// Every member obstacle index, `frame` first.
    pub members: Vec<usize>,
    pub kind: PlanKind,
    pub reason: String,
    /// Body properties of a dynamic unit.
    pub props: BodyProps,
}

/// Part categories that mean *bolted down*: a unit pinned to one stays a
/// kinematic mirror under `anchored` (the default).
const EQUIPMENT_CATEGORIES: &[&str] = &[
    "structure.",
    "machine_tool",
    "conveyor",
    "fixture.",
    "sensor.",
    "hmi.",
    "io.",
    "power_supply",
    "robot_controller",
    "vehicle.",
    "feeder.",
];

/// Part categories that mean *loose*: stock, carriers and packaging —
/// what a cell handles rather than what it is built of. A group pinned to
/// anything else (a washer, a gantry, a stand nobody catalogued) is
/// equipment too: a purchasable assembly is bolted down unless it is one
/// of these.
const LOOSE_CATEGORIES: &[&str] = &[
    "workpiece",
    "pallet",
    "tray",
    "carton",
    "tote",
    "bin",
    "box",
    "unit_load",
    "part",
    "stock",
    "package",
];

/// How far under the ground plane a unit's lowest point may sit before
/// it counts as floor rather than a body to eject (m).
const BURIED_MARGIN: f64 = 0.01;

/// How deep two bounding boxes must overlap, in every axis, before a
/// dynamic unit is reported as starting inside something (m): contact
/// slop and the 5 mm "float a placed part" convention stay quiet.
const OVERLAP_MARGIN: f64 = 0.005;

fn is_equipment(category: &str) -> bool {
    EQUIPMENT_CATEGORIES
        .iter()
        .any(|prefix| category.starts_with(prefix))
}

fn is_loose(category: &str) -> bool {
    LOOSE_CATEGORIES
        .iter()
        .any(|c| category == *c || category.starts_with(&format!("{c}.")))
}

/// Proper prefixes of a `/`-separated name, longest first.
fn prefixes(name: &str) -> impl Iterator<Item = &str> {
    name.rmatch_indices('/').map(move |(i, _)| &name[..i])
}

/// The units of the scene under `options`, in obstacle order of their
/// frame member. Declared scope: one unit per obstacle marked dynamic and
/// nothing else (the rest are mirrors). World scope: every enabled-or-
/// walkable obstacle belongs to exactly one unit.
pub(crate) fn derive_units(scene: &Scene, options: &PhysicsOptions) -> Vec<Unit> {
    let obstacles = scene.obstacles();
    let mut units = Vec::new();
    if options.scope == PhysicsScope::Declared {
        for (i, o) in obstacles.iter().enumerate() {
            if !o.enabled {
                continue;
            }
            let Some(props) = scene.resolved_body_props(&o.name) else {
                continue;
            };
            if props.kind != BodyKind::Dynamic {
                continue;
            }
            units.push(Unit {
                name: o.name.clone(),
                frame: i,
                members: vec![i],
                kind: PlanKind::Dynamic,
                reason: "declared dynamic".to_string(),
                props,
            });
        }
        return units;
    }

    let index_of: HashMap<&str, usize> = obstacles
        .iter()
        .enumerate()
        .map(|(i, o)| (o.name.as_str(), i))
        .collect();
    // Every part pin names a resident whose obstacles live under
    // `<target>/`: a group or an obstacle, but also a sensor's housing, a
    // device's frame, a camera's fixture. Any of them roots a unit.
    let pinned: HashSet<&str> = scene.parts().iter().map(|p| p.target.as_str()).collect();
    let mut listed: HashSet<&str> = HashSet::new();
    for device in scene.devices() {
        let names: Vec<&String> = match &device.kind {
            DeviceKind::LinearAxis { objects, .. } => objects.iter().collect(),
            DeviceKind::Vehicle { body, .. } => body.iter().collect(),
            DeviceKind::Lift { car, .. } => car.iter().collect(),
            DeviceKind::Source { pool, .. } => pool.iter().collect(),
            DeviceKind::Conveyor { .. } | DeviceKind::Sink { .. } => Vec::new(),
        };
        listed.extend(names.into_iter().map(String::as_str));
    }
    // What a program picks up *on its own* is a thing in its own right —
    // a case off a stack, a part out of a tray — and roots a unit. What a
    // program takes *together* in one step (every board, block and case
    // of a pallet, attached to a lift's platform at once) is a unit
    // carried whole: those pieces stay in the unit their names put them
    // in, rigid to each other, and the lift takes the unit. Splitting
    // them would leave a pallet's nailed deck boards free bodies —
    // measured: they fell ten centimetres and the load toppled.
    let mut grasped: HashSet<String> = HashSet::new();
    fn grasped_alone(steps: &[crate::seq::Step], out: &mut HashSet<String>) {
        for step in steps {
            let attached: Vec<&String> = step
                .actions
                .iter()
                .filter_map(|action| match action {
                    Action::Attach { object, .. } => Some(object),
                    _ => None,
                })
                .collect();
            if let [only] = attached.as_slice() {
                out.insert((*only).clone());
            }
            for arm in &step.select {
                grasped_alone(&arm.steps, out);
            }
        }
    }
    for sequence in scene.sequences() {
        grasped_alone(&sequence.steps, &mut grasped);
    }
    // An obstacle that starts its own unit whatever hangs above it: one
    // with physics of its own, a floor, a device's, a program's, or one
    // identified as a thing in its own right (an obstacle part pin — a
    // crate inside a pallet's scope is a crate, not a deck board).
    let obstacle_pins: HashSet<&str> = scene
        .parts()
        .iter()
        .filter(|p| p.kind == PartTargetKind::Obstacle)
        .map(|p| p.target.as_str())
        .collect();
    let is_cut = |i: usize| {
        let o = &obstacles[i];
        o.physics.is_some()
            || o.walkable
            || listed.contains(o.name.as_str())
            || grasped.contains(&o.name)
            || obstacle_pins.contains(o.name.as_str())
    };
    let parent_key = |name: &str| -> Option<String> {
        prefixes(name)
            .find(|p| index_of.contains_key(p) || pinned.contains(p))
            .map(str::to_string)
    };
    let root_key = |i: usize| -> String {
        let mut cur = obstacles[i].name.clone();
        loop {
            if let Some(&k) = index_of.get(cur.as_str()) {
                if is_cut(k) {
                    return cur;
                }
            }
            match parent_key(&cur) {
                Some(p) => cur = p,
                None => return cur,
            }
        }
    };
    // Members per unit key, in obstacle order (the first is the frame).
    let mut members: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    let mut order: Vec<String> = Vec::new();
    for i in 0..obstacles.len() {
        let key = root_key(i);
        let entry = members.entry(key.clone()).or_default();
        if entry.is_empty() {
            order.push(key);
        }
        entry.push(i);
    }
    for key in order {
        let member_indices = members.remove(&key).expect("unit has members");
        let frame = match index_of.get(key.as_str()) {
            Some(&i) => i,
            None => member_indices[0],
        };
        let mut ordered = vec![frame];
        ordered.extend(member_indices.iter().copied().filter(|&i| i != frame));
        let has_collider = ordered
            .iter()
            .any(|&i| obstacles[i].enabled || obstacles[i].walkable);
        let pin = scene.part(&key);
        let category = pin.and_then(|p| p.part.category.clone());
        let pin_mass = pin.and_then(|p| {
            p.part
                .attributes
                .get("mass_kg")
                .and_then(PartAttr::as_number)
                .filter(|kg| kg.is_finite() && *kg > 0.0)
        });
        let root = index_of.get(key.as_str()).copied();
        // The unit's lowest collision point: a slab authored under the
        // ground plane (the usual `floor` box at z = -0.05) is part of
        // the floor, not a loose body for the ground to eject.
        let lowest = ordered
            .iter()
            .filter(|&&i| obstacles[i].enabled || obstacles[i].walkable)
            .filter_map(|&i| scene.obstacle_colliders()[i].aabb(&obstacles[i].pose))
            .map(|(min, _)| min[2])
            .fold(f64::INFINITY, f64::min);
        let buried = options
            .ground
            .is_some_and(|g| lowest.is_finite() && lowest < g - BURIED_MARGIN);
        let (kind, reason, props) = if !has_collider {
            (
                PlanKind::Fixed,
                "no collision geometry (not lowered)".to_string(),
                BodyProps::default(),
            )
        } else if let Some(props) = root.and_then(|i| {
            obstacles[i].physics.as_ref().map(|_| {
                scene
                    .resolved_body_props(&obstacles[i].name)
                    .unwrap_or_default()
            })
        }) {
            if props.kind == BodyKind::Dynamic {
                (PlanKind::Dynamic, "declared dynamic".to_string(), props)
            } else {
                (PlanKind::Fixed, "declared static".to_string(), props)
            }
        } else if root.is_some_and(|i| listed.contains(obstacles[i].name.as_str())) {
            (
                PlanKind::Fixed,
                "moved by a device".to_string(),
                BodyProps::default(),
            )
        } else if root.is_some_and(|i| obstacles[i].walkable) {
            (
                PlanKind::Fixed,
                "walkable floor".to_string(),
                BodyProps::default(),
            )
        } else if root.is_some_and(|i| grasped.contains(&obstacles[i].name)) {
            (
                PlanKind::Dynamic,
                "grasped by a program".to_string(),
                dynamic_props(pin_mass),
            )
        } else if buried {
            (
                PlanKind::Fixed,
                "below the ground (a floor slab)".to_string(),
                BodyProps::default(),
            )
        } else if options.anchored && anchored_by_pin(pin, category.as_deref()) {
            (
                PlanKind::Fixed,
                format!("equipment ({})", category.as_deref().unwrap_or("unnamed")),
                BodyProps::default(),
            )
        } else {
            let reason = match &category {
                Some(c) => format!("loose ({c})"),
                None => "loose (unpinned)".to_string(),
            };
            (PlanKind::Dynamic, reason, dynamic_props(pin_mass))
        };
        units.push(Unit {
            name: key,
            frame,
            members: ordered,
            kind,
            reason,
            props,
        });
    }
    units.sort_by_key(|u| u.frame);
    units
}

/// Whether a part pin holds its unit fixed: an equipment category on any
/// pin, or a group pin to anything but a loose category (an assembly
/// someone catalogued is bolted down unless it is stock or a carrier).
fn anchored_by_pin(pin: Option<&crate::part::PartEntry>, category: Option<&str>) -> bool {
    match (pin, category) {
        (_, Some(c)) if is_equipment(c) => true,
        (_, Some(c)) if is_loose(c) => false,
        // A bare obstacle pin is stock unless its category says otherwise;
        // every other pin kind (a group, a sensor, a device, a camera, a
        // LiDAR, a robot) names equipment.
        (Some(entry), _) => entry.kind != PartTargetKind::Obstacle,
        (None, _) => false,
    }
}

fn dynamic_props(mass: Option<f64>) -> BodyProps {
    BodyProps {
        mass,
        ..BodyProps::dynamic()
    }
}

impl Scene {
    /// The audit of what a physics bake under `options` would hand the
    /// engine (design-world-physics.md §3.2): the ground, every rigid
    /// unit with its kind and the rule that decided it, and the count of
    /// obstacles mirrored individually. Robots are not listed here (their
    /// lowering is the declaration's, `set_robot_dynamics`).
    pub fn physics_plan(&self, options: &PhysicsOptions) -> PhysicsPlan {
        let units = derive_units(self, options);
        let mut rows = Vec::new();
        if let Some(z) = options.ground {
            rows.push(PlannedBody {
                name: "ground".to_string(),
                kind: PlanKind::Ground,
                members: Vec::new(),
                reason: format!("half-space at z = {z}"),
                mass: None,
            });
        }
        let mut covered = 0usize;
        for unit in &units {
            covered += unit
                .members
                .iter()
                .filter(|&&i| self.obstacles()[i].enabled)
                .count();
            rows.push(PlannedBody {
                name: unit.name.clone(),
                kind: unit.kind,
                members: unit
                    .members
                    .iter()
                    .map(|&i| self.obstacles()[i].name.clone())
                    .collect(),
                reason: unit.reason.clone(),
                mass: if unit.kind == PlanKind::Dynamic {
                    unit.props.mass
                } else {
                    None
                },
            });
        }
        for (r, sr) in self.robots().iter().enumerate() {
            let Some((dynamics, defaulted)) = robot_dynamics_for(self, r, options) else {
                continue;
            };
            let floating = robot_floating(self, r, options);
            let joints = match options.powered {
                Some(true) => "servo",
                Some(false) => "passive",
                None => "servo under a program that drives it, passive without",
            };
            let base = if floating {
                "floating"
            } else if sr.mount.is_some() {
                "on its vehicle"
            } else {
                "on its stand"
            };
            // A rolling machine's wheels (and their steering) are its
            // mount's: kinematic, turned by what the vehicle drives.
            let rolled: usize = sr
                .mount
                .as_ref()
                .and_then(|m| m.drive.as_ref())
                .map_or(0, |d| {
                    d.wheels
                        .iter()
                        .map(|w| 1 + usize::from(w.steer.is_some()))
                        .sum()
                });
            let mut reason = format!(
                "{} joints {joints}, base {base}",
                dynamics.servos.len().saturating_sub(rolled)
            );
            if rolled > 0 {
                reason.push_str(&format!(", {rolled} wheel joint(s) turned by the vehicle"));
            }
            if defaulted {
                reason.push_str(&format!(
                    ", force cap defaulted to {} N·m (the model states no plausible effort limit)",
                    crate::dynamics::DEFAULT_WORLD_CAP_REVOLUTE
                ));
            }
            let stated: f64 = sr
                .model
                .links
                .iter()
                .filter_map(|l| l.inertial.as_ref().map(|i| i.mass))
                .sum();
            rows.push(PlannedBody {
                name: sr.name.clone(),
                kind: PlanKind::Robot,
                members: Vec::new(),
                reason,
                mass: (stated > 0.0).then_some(stated),
            });
        }
        let enabled = self.obstacles().iter().filter(|o| o.enabled).count();
        PhysicsPlan {
            rows,
            mirrors: enabled.saturating_sub(covered),
            overlaps: self.unit_overlaps(&units),
        }
    }

    /// Dynamic units whose collision parts interpenetrate an enabled
    /// obstacle outside the unit at the authored poses: an AABB pass over
    /// the scene, then the exact test on the candidates.
    fn unit_overlaps(&self, units: &[Unit]) -> Vec<(String, String)> {
        let obstacles = self.obstacles();
        let colliders = self.obstacle_colliders();
        let boxes: Vec<Option<([f64; 3], [f64; 3])>> = obstacles
            .iter()
            .enumerate()
            .map(|(i, o)| {
                if o.enabled {
                    colliders[i].aabb(&o.pose)
                } else {
                    None
                }
            })
            .collect();
        let mut unit_of: HashMap<usize, usize> = HashMap::new();
        for (k, unit) in units.iter().enumerate() {
            for &i in &unit.members {
                unit_of.insert(i, k);
            }
        }
        let mut out = Vec::new();
        for (k, unit) in units.iter().enumerate() {
            if unit.kind != PlanKind::Dynamic {
                continue;
            }
            let mut hits: Vec<usize> = Vec::new();
            for &m in &unit.members {
                let Some((lo, hi)) = boxes[m] else {
                    continue;
                };
                for (j, other) in boxes.iter().enumerate() {
                    let Some((olo, ohi)) = other else {
                        continue;
                    };
                    if unit_of.get(&j) == Some(&k) || hits.contains(&j) {
                        continue;
                    }
                    // Touching faces are contact, not penetration: only
                    // boxes that overlap by more than the margin in every
                    // axis are candidates.
                    let overlap =
                        (0..3).all(|a| hi[a].min(ohi[a]) - lo[a].max(olo[a]) > OVERLAP_MARGIN);
                    if overlap
                        && colliders[m].intersects(
                            &obstacles[m].pose,
                            &colliders[j],
                            &obstacles[j].pose,
                        )
                    {
                        hits.push(j);
                    }
                }
            }
            hits.sort_unstable();
            for j in hits {
                out.push((unit.name.clone(), obstacles[j].name.clone()));
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::part::Part;
    use crate::seq::{Condition, Device, Sequence, Step};
    use botrail_model::Geometry;
    use nalgebra::{Isometry3, Vector3};

    fn boxed(scene: &mut Scene, name: &str, x: f64, z: f64) {
        scene
            .add_obstacle(
                name,
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                Isometry3::translation(x, 0.0, z),
            )
            .unwrap();
    }

    fn pin(
        scene: &mut Scene,
        target: &str,
        kind: PartTargetKind,
        category: &str,
        mass: Option<f64>,
    ) {
        let mut part = Part {
            catalog: None,
            manufacturer: None,
            model: None,
            category: Some(category.to_string()),
            description: None,
            qty: 1,
            attributes: Default::default(),
        };
        if let Some(kg) = mass {
            part.attributes
                .insert("mass_kg".to_string(), PartAttr::Number(kg));
        }
        scene.set_part(target, Some(kind), part).unwrap();
    }

    fn cell() -> Scene {
        let mut scene = Scene::empty();
        // A pinned rack: two pieces, equipment → fixed as one unit.
        boxed(&mut scene, "rack/post", 0.0, 0.5);
        boxed(&mut scene, "rack/shelf", 0.0, 1.0);
        pin(
            &mut scene,
            "rack",
            PartTargetKind::Group,
            "structure.rack",
            None,
        );
        // A pallet pinned with its mass, a disabled visual timber under it.
        boxed(&mut scene, "pallet/slab", 1.0, 0.07);
        boxed(&mut scene, "pallet/visual/board", 1.0, 0.05);
        scene
            .set_obstacle_enabled("pallet/visual/board", false)
            .unwrap();
        pin(
            &mut scene,
            "pallet",
            PartTargetKind::Group,
            "pallet",
            Some(18.0),
        );
        // An unpinned box on the floor, and its own child decal.
        boxed(&mut scene, "crate", 2.0, 0.05);
        boxed(&mut scene, "crate/lid", 2.0, 0.11);
        // A walkable slab that is out of collision (a floor).
        boxed(&mut scene, "slab", 3.0, -0.05);
        scene.set_obstacle_walkable("slab", true).unwrap();
        scene.set_obstacle_enabled("slab", false).unwrap();
        // A door panel an axis drives, with a handle hanging under it.
        boxed(&mut scene, "door/panel", 4.0, 1.0);
        boxed(&mut scene, "door/panel/handle", 4.0, 1.0);
        scene.upsert_device(Device {
            name: "door".into(),
            kind: DeviceKind::LinearAxis {
                objects: vec!["door/panel".into()],
                axis: nalgebra::Unit::new_normalize(Vector3::x()),
                speed: 0.1,
                position: 0.0,
                range: (0.0, 1.0),
                stops: Vec::new(),
            },
        });
        // Declared bodies keep their word.
        boxed(&mut scene, "anvil", 5.0, 2.0);
        scene
            .set_obstacle_physics("anvil", Some(BodyProps::default()))
            .unwrap();
        boxed(&mut scene, "ball", 6.0, 2.0);
        scene
            .set_obstacle_physics(
                "ball",
                Some(BodyProps {
                    mass: Some(0.5),
                    ..BodyProps::dynamic()
                }),
            )
            .unwrap();
        // A basket inside the washer's group that a program grasps: cut
        // out of the equipment, dynamic on its own with its mesh.
        boxed(&mut scene, "washer/tank", 7.0, 0.5);
        boxed(&mut scene, "washer/basket", 7.0, 0.8);
        boxed(&mut scene, "washer/basket/mesh", 7.0, 0.8);
        scene
            .set_obstacle_enabled("washer/basket/mesh", false)
            .unwrap();
        pin(&mut scene, "washer", PartTargetKind::Group, "washer", None);
        scene.upsert_sequence(Sequence {
            name: "wash".into(),
            steps: vec![Step {
                name: "grab".into(),
                actions: vec![Action::Attach {
                    robot: None,
                    object: "washer/basket".into(),
                    link: None,
                    touch_links: None,
                    group: None,
                }],
                transition: Condition::Elapsed { seconds: 1.0 },
                select: Vec::new(),
            }],
        });
        // Floor tape: disabled, unpinned, alone — never lowered.
        boxed(&mut scene, "tape", 8.0, 0.0);
        scene.set_obstacle_enabled("tape", false).unwrap();
        // The usual floor box, authored under the ground plane.
        boxed(&mut scene, "subfloor", 9.0, -0.06);
        // A tray: a pinned group, but a carrier — loose.
        boxed(&mut scene, "tray/base", 10.0, 0.8);
        boxed(&mut scene, "tray/insert", 10.0, 0.85);
        scene.set_obstacle_enabled("tray/insert", false).unwrap();
        pin(&mut scene, "tray", PartTargetKind::Group, "tray", None);
        // A crate inside the pallet's scope, pinned as a thing of its own.
        boxed(&mut scene, "pallet/crate", 1.0, 0.19);
        pin(
            &mut scene,
            "pallet/crate",
            PartTargetKind::Obstacle,
            "workpiece",
            Some(3.0),
        );
        // A part authored 3 cm into the bench it sits on.
        boxed(&mut scene, "bench", 11.0, 0.4);
        boxed(&mut scene, "sunk", 11.0, 0.42);
        // A photoelectric sensor: its housing hangs under a *sensor* pin.
        boxed(&mut scene, "eye/body", 12.0, 0.9);
        boxed(&mut scene, "eye/bracket", 12.0, 0.85);
        scene
            .upsert_sensor(crate::seq::Sensor {
                name: "eye".into(),
                kind: crate::seq::SensorKind::Zone {
                    pose: Isometry3::identity(),
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                watch: crate::seq::SensorWatch::AllObjects,
                mount: None,
            })
            .unwrap();
        pin(
            &mut scene,
            "eye",
            PartTargetKind::Sensor,
            "sensor.photoelectric",
            None,
        );
        scene
    }

    fn row<'a>(plan: &'a PhysicsPlan, name: &str) -> &'a PlannedBody {
        plan.rows
            .iter()
            .find(|r| r.name == name)
            .unwrap_or_else(|| panic!("no row `{name}` in {:?}", plan.rows))
    }

    #[test]
    fn world_scope_folds_the_cell_into_units_and_decides_each() {
        let scene = cell();
        let plan = scene.physics_plan(&PhysicsOptions::world());
        assert_eq!(plan.rows[0].kind, PlanKind::Ground);
        let rack = row(&plan, "rack");
        assert_eq!(rack.kind, PlanKind::Fixed);
        assert_eq!(rack.members, vec!["rack/post", "rack/shelf"]);
        assert!(rack.reason.starts_with("equipment"), "{}", rack.reason);
        let pallet = row(&plan, "pallet");
        assert_eq!(pallet.kind, PlanKind::Dynamic);
        assert_eq!(pallet.members, vec!["pallet/slab", "pallet/visual/board"]);
        assert_eq!(pallet.mass, Some(18.0));
        let crate_ = row(&plan, "pallet/crate");
        assert_eq!(crate_.kind, PlanKind::Dynamic);
        assert_eq!(crate_.members, vec!["pallet/crate"]);
        assert_eq!(crate_.mass, Some(3.0));
        // The sunk part starts inside the bench: reported, both ways
        // round (both are loose), and nothing else.
        assert_eq!(
            plan.overlaps,
            vec![
                ("bench".to_string(), "sunk".to_string()),
                ("sunk".to_string(), "bench".to_string())
            ]
        );
        assert!(plan.to_markdown().contains("- sunk × bench"));
        let crate_ = row(&plan, "crate");
        assert_eq!(crate_.kind, PlanKind::Dynamic);
        assert_eq!(crate_.members, vec!["crate", "crate/lid"]);
        assert_eq!(crate_.mass, None);
        assert_eq!(row(&plan, "slab").kind, PlanKind::Fixed);
        let door = row(&plan, "door/panel");
        assert_eq!(door.kind, PlanKind::Fixed);
        assert_eq!(door.members, vec!["door/panel", "door/panel/handle"]);
        assert_eq!(row(&plan, "anvil").kind, PlanKind::Fixed);
        assert_eq!(row(&plan, "anvil").reason, "declared static");
        assert_eq!(row(&plan, "ball").kind, PlanKind::Dynamic);
        assert_eq!(row(&plan, "ball").mass, Some(0.5));
        let washer = row(&plan, "washer");
        assert_eq!(washer.kind, PlanKind::Fixed, "{}", washer.reason);
        assert_eq!(washer.members, vec!["washer/tank"]);
        assert_eq!(washer.reason, "equipment (washer)");
        // A group pinned to a carrier category is loose, mass and all.
        let tray = row(&plan, "tray");
        assert_eq!(tray.kind, PlanKind::Dynamic, "{}", tray.reason);
        assert_eq!(tray.reason, "loose (tray)");
        let eye = row(&plan, "eye");
        assert_eq!(eye.kind, PlanKind::Fixed, "{}", eye.reason);
        assert_eq!(eye.members, vec!["eye/body", "eye/bracket"]);
        let basket = row(&plan, "washer/basket");
        assert_eq!(basket.kind, PlanKind::Dynamic);
        assert_eq!(basket.members, vec!["washer/basket", "washer/basket/mesh"]);
        assert_eq!(basket.reason, "grasped by a program");
        let tape = row(&plan, "tape");
        assert_eq!(tape.kind, PlanKind::Fixed);
        assert!(tape.reason.contains("not lowered"));
        let subfloor = row(&plan, "subfloor");
        assert_eq!(subfloor.kind, PlanKind::Fixed);
        assert!(
            subfloor.reason.contains("below the ground"),
            "{}",
            subfloor.reason
        );
        // Without a ground plane nothing is buried: the slab is loose.
        let no_ground = scene.physics_plan(&PhysicsOptions {
            ground: None,
            ..PhysicsOptions::world()
        });
        assert_eq!(row(&no_ground, "subfloor").kind, PlanKind::Dynamic);
        // Every enabled obstacle is in some unit: nothing mirrored blindly.
        assert_eq!(plan.mirrors, 0);
        // With anchoring off, the rack is loose too.
        let loose = scene.physics_plan(&PhysicsOptions {
            anchored: false,
            ..PhysicsOptions::world()
        });
        assert_eq!(row(&loose, "rack").kind, PlanKind::Dynamic);
        let md = plan.to_markdown();
        assert!(md.contains("| pallet | dynamic | 2 | 18 |"), "{md}");
    }

    #[test]
    fn a_robot_is_a_row_under_the_world_scope_and_when_declared() {
        let scene = crate::dynamics::tests_support::arm(false);
        let plan = scene.physics_plan(&PhysicsOptions::world());
        let arm = row(&plan, "simple_arm");
        assert_eq!(arm.kind, PlanKind::Robot);
        assert!(
            arm.reason.starts_with(
                "6 joints servo under a program that drives it, passive without, base on its stand"
            ),
            "{}",
            arm.reason
        );
        assert!(!arm.reason.contains("defaulted"), "{}", arm.reason);
        let powered = scene.physics_plan(&PhysicsOptions {
            powered: Some(false),
            ..PhysicsOptions::world()
        });
        assert!(row(&powered, "simple_arm")
            .reason
            .starts_with("6 joints passive"));
        // Declared scope: an undeclared robot is a mirror, so no row.
        let declared = scene.physics_plan(&PhysicsOptions::default());
        assert!(!declared.rows.iter().any(|r| r.name == "simple_arm"));
        let mut declared_scene = crate::dynamics::tests_support::arm(true);
        declared_scene
            .set_robot_dynamics_with(0, true, None, None, None, None, Some(true))
            .unwrap();
        let plan = declared_scene.physics_plan(&PhysicsOptions::default());
        assert!(row(&plan, "simple_arm").reason.contains("base floating"));
    }

    #[test]
    fn declared_scope_lists_only_what_was_marked() {
        let scene = cell();
        let plan = scene.physics_plan(&PhysicsOptions::default());
        assert_eq!(plan.rows.len(), 1, "{:?}", plan.rows);
        assert_eq!(plan.rows[0].name, "ball");
        assert_eq!(plan.rows[0].reason, "declared dynamic");
        // Everything else enabled is a mirror.
        let enabled = scene.obstacles().iter().filter(|o| o.enabled).count();
        assert_eq!(plan.mirrors, enabled - 1);
    }
}
