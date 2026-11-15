//! The joints of a prop imported as scenery: a cabinet's door, a
//! refrigerator's drawers, a dishwasher's racks.
//!
//! With [`crate::ImportOptions::articulations_as_scenery`] an articulated
//! prop comes in as obstacles at their authored poses. Its revolute and
//! prismatic joints are not thrown away: each is reported as an
//! [`ImportedJoint`] — the axis in the world, the range, the obstacles
//! that move on it (the child body's geometry and everything jointed on
//! below it) and an obstacle of the body it hangs on — for the scene to
//! keep as a scenery joint. The authored pose counts as position 0.

use std::collections::{HashSet, VecDeque};

use nalgebra::{Point3, UnitQuaternion, Vector3};
use openusd::schemas::physics::{self, JointBase};
use openusd::usd::Prim;
use openusd::{gf, tf};

use crate::articulation::AnyJoint;
use crate::{decompose_matrix, Importer};

/// A door or a drawer of a prop imported as scenery.
#[derive(Debug, Clone)]
pub struct ImportedJoint {
    /// The joint prim's path.
    pub name: String,
    /// A revolute joint (the range is in radians); otherwise prismatic (metres).
    pub hinge: bool,
    /// A point on the axis and its direction: world, metres, Z-up.
    pub origin: Point3<f64>,
    pub axis: Vector3<f64>,
    /// `(lower, upper)` as authored, `None` when the joint has no limits.
    pub range: Option<(f64, f64)>,
    /// A node of the body the joint hangs on (`None`: nothing to hang on).
    pub base: Option<String>,
    /// The nodes that move with it.
    pub objects: Vec<String>,
}

/// A joint prim as read: the two bodies and, for a joint that moves, where.
struct Edge {
    name: String,
    body0: Option<String>,
    body1: String,
    /// `Some(hinge)` for a revolute / prismatic joint, `None` for a weld.
    hinge: Option<bool>,
    pos0: Vector3<f64>,
    rot0: UnitQuaternion<f64>,
    axis: Vector3<f64>,
    lower: Option<f64>,
    upper: Option<f64>,
}

impl Importer<'_> {
    /// Fills [`crate::ImportedScene::joints`] from the joint prims the walk
    /// met. Runs after it: the members are the nodes it produced.
    pub(crate) fn collect_scenery_joints(&mut self) {
        let prims = std::mem::take(&mut self.joint_prims);
        let mut edges = Vec::new();
        for (prim, path, type_name) in prims {
            match self.read_edge(&prim, &path, &type_name) {
                Ok(Some(edge)) => edges.push(edge),
                Ok(None) => {}
                Err(e) => self.out.warnings.push(format!("{path}: {e}")),
            }
        }
        let under = |node: &str, body: &str| {
            node == body
                || node
                    .strip_prefix(body)
                    .is_some_and(|rest| rest.starts_with('/'))
        };
        for edge in edges.iter().filter(|e| e.hinge.is_some()) {
            // The child body and every body jointed on below it.
            let mut bodies = vec![edge.body1.as_str()];
            let mut seen: HashSet<&str> = HashSet::from([edge.body1.as_str()]);
            let mut queue = VecDeque::from([edge.body1.as_str()]);
            while let Some(body) = queue.pop_front() {
                for next in edges.iter().filter(|e| e.body0.as_deref() == Some(body)) {
                    if seen.insert(next.body1.as_str()) {
                        bodies.push(next.body1.as_str());
                        queue.push_back(next.body1.as_str());
                    }
                }
            }
            let moves = |node: &str| bodies.iter().any(|body| under(node, body));
            let objects: Vec<String> = self
                .out
                .nodes
                .iter()
                .filter(|n| moves(&n.name))
                .map(|n| n.name.clone())
                .collect();
            if objects.is_empty() {
                self.out.warnings.push(format!(
                    "{}: nothing drawn moves on this joint; left out",
                    edge.name
                ));
                continue;
            }
            // What it hangs on: a node of the parent body, else of anything
            // that stays — so the axis follows the prop when it is placed.
            let stays = |n: &&crate::ImportedNode| !moves(&n.name);
            let base = edge
                .body0
                .as_deref()
                .and_then(|body| {
                    self.out
                        .nodes
                        .iter()
                        .filter(stays)
                        .find(|n| under(&n.name, body))
                })
                .or_else(|| self.out.nodes.iter().find(stays))
                .map(|n| n.name.clone());

            let world0 = edge
                .body0
                .as_deref()
                .and_then(|body| self.worlds.get(body))
                .copied()
                .unwrap_or_default();
            let (frame, residual) = decompose_matrix(&world0);
            let origin = frame * Point3::from(residual * edge.pos0);
            let axis = frame.rotation * (edge.rot0 * edge.axis);
            let hinge = edge.hinge == Some(true);
            let unit = if hinge {
                1f64.to_radians()
            } else {
                self.meters_per_unit
            };
            self.out.joints.push(ImportedJoint {
                name: edge.name.clone(),
                hinge,
                origin: Point3::from(self.up_axis_fix * (origin.coords * self.meters_per_unit)),
                axis: self.up_axis_fix * axis,
                range: edge
                    .lower
                    .zip(edge.upper)
                    .map(|(lo, hi)| (lo * unit, hi * unit)),
                base,
                objects,
            });
        }
    }

    fn read_edge(&self, prim: &Prim, path: &str, type_name: &str) -> anyhow::Result<Option<Edge>> {
        let hinge = match type_name {
            "PhysicsRevoluteJoint" => Some(true),
            "PhysicsPrismaticJoint" => Some(false),
            "PhysicsFixedJoint" => None,
            _ => return Ok(None),
        };
        if prim
            .attribute("physics:jointEnabled")
            .get::<bool>()
            .ok()
            .flatten()
            == Some(false)
        {
            return Ok(None);
        }
        let joint = AnyJoint(prim.clone());
        let target = |rel: openusd::usd::Relationship| -> anyhow::Result<Option<String>> {
            Ok(rel.targets()?.first().map(|p| p.to_string()))
        };
        let body0 = target(joint.body0_rel())?;
        let Some(body1) = target(joint.body1_rel())? else {
            return Ok(None);
        };
        let pos0 = joint
            .local_pos0_attr()
            .get::<[f32; 3]>()?
            .map(|p| Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64))
            .unwrap_or_else(Vector3::zeros);
        let rot0 = joint
            .local_rot0_attr()
            .get::<gf::Quatf>()?
            .map(|q| {
                UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    q.w as f64, q.x as f64, q.y as f64, q.z as f64,
                ))
            })
            .unwrap_or_else(UnitQuaternion::identity);
        let (axis, lower, upper) = match hinge {
            Some(true) => {
                let v = physics::RevoluteJoint::get(self.stage, prim.path().clone())?
                    .ok_or_else(|| anyhow::anyhow!("not a revolute joint"))?;
                (
                    v.axis_attr().get::<tf::Token>()?,
                    v.lower_limit_attr().get::<f32>()?.map(f64::from),
                    v.upper_limit_attr().get::<f32>()?.map(f64::from),
                )
            }
            Some(false) => {
                let v = physics::PrismaticJoint::get(self.stage, prim.path().clone())?
                    .ok_or_else(|| anyhow::anyhow!("not a prismatic joint"))?;
                (
                    v.axis_attr().get::<tf::Token>()?,
                    v.lower_limit_attr().get::<f32>()?.map(f64::from),
                    v.upper_limit_attr().get::<f32>()?.map(f64::from),
                )
            }
            None => (None, None, None),
        };
        let axis = match axis.as_ref().map(|t| t.as_str()) {
            Some("Y") => Vector3::y(),
            Some("Z") => Vector3::z(),
            // UsdPhysics' fallback for both joint types.
            _ => Vector3::x(),
        };
        Ok(Some(Edge {
            name: path.to_string(),
            body0,
            body1,
            hinge,
            pos0,
            rot0,
            axis,
            lower,
            upper,
        }))
    }
}
