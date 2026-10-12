//! Cable and tube routes (design-routing.md). A route is authored once —
//! which connection it carries, which line it is made of, which way points,
//! ducts and clamps it passes — and everything else is derived every time it
//! is read, from the residents' *current* poses: the centerline with its
//! bends, the cut and purchase length, the BOM rows, the cable list and the
//! static checks. Move the cabinet and the cable follows, and so does its
//! length.
//!
//! Nothing here runs the cell. Fixed spans are a closed-form polyline with
//! fillets of the line's minimum bend radius; moving spans (between a clamp
//! on a robot link and the next anchor) are drawn at their time-zero shape
//! and verified by the rope replay (R2).
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

use nalgebra::{Isometry3, Point3, Rotation3, Unit, Vector3};
use serde::{Deserialize, Serialize};

use crate::connections::{Attach, Connection, Medium, Port, PortGeometry};
use crate::part::{BomRow, BomTarget, CatalogRef, PartAttr, PartTargetKind};
use crate::rope::RopeTrack;
use crate::Scene;

fn default_true() -> bool {
    true
}

/// What a line is: a cable (power, signal, network), a pneumatic tube or a
/// hose. Decides the BOM category with the carried medium.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum LineKind {
    #[default]
    Cable,
    Tube,
    Hose,
}

impl LineKind {
    pub fn as_str(self) -> &'static str {
        match self {
            LineKind::Cable => "cable",
            LineKind::Tube => "tube",
            LineKind::Hose => "hose",
        }
    }
}

/// The line a route is made of — a rule several routes share: change the
/// spec and every route's bends, lengths and BOM rows follow. Values are
/// declared, never guessed: a `None` reads as unknown in the checks.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct CableSpec {
    pub name: String,
    pub kind: LineKind,
    /// The media this line may carry; empty = not stated (no check).
    pub media: Vec<Medium>,
    /// Outer diameter (m).
    pub od_m: Option<f64>,
    /// Minimum bend radius when fixed (m): the fillet at every corner.
    pub bend_r_fixed_m: Option<f64>,
    /// Minimum bend radius in continuous motion (m): the moving-span check.
    pub bend_r_moving_m: Option<f64>,
    pub mass_kg_m: Option<f64>,
    /// A product or standard limit on one run (a 100 m Ethernet segment).
    pub max_length_m: Option<f64>,
    /// Torsion the line tolerates, degrees per metre of free length (a
    /// moving span's twist between its holders is checked against it).
    pub torsion_deg_m: Option<f64>,
    /// Extra material at each end (stripping, the connector).
    pub end_allowance_m: f64,
    /// Lengths the line is sold in (a 6 m or 12 m robot cable); the purchase
    /// length is the shortest that fits.
    pub sold_lengths_m: Vec<f64>,
    /// Sold by the reel / cut to a step (20 m reels, 0.5 m steps).
    pub order_step_m: Option<f64>,
    pub manufacturer: Option<String>,
    pub model: Option<String>,
    /// A catalog reference, `id` or `id@revision`.
    pub catalog: Option<String>,
    /// Display colour, linear RGB; `None` draws the medium's default.
    pub color: Option<[f32; 3]>,
}

/// One way point of a route: a free point in space, or a support (a clamp
/// the route is held at, a duct it runs through).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum Via {
    Point {
        position: [f64; 3],
    },
    Support {
        name: String,
    },
    /// Run along a body: auto clamps every `pitch_m` (the route's
    /// `clamp_pitch_m` when unset) on the face of the body's collision
    /// bounds nearest the way in and the way out, the line held off the
    /// surface by its radius and the clamp's standoff.
    Along {
        on: Body,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        pitch_m: Option<f64>,
        /// The face to lay the run on, in the body's frame (`+x` … `-z`);
        /// the side face nearest the way in and the way out when unset.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face: Option<String>,
        /// How far the run is moved sideways on its face (m), along the
        /// body's axis across the run, toward that axis's positive end —
        /// two lines along one face, side by side, each with its clips.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        offset_m: Option<f64>,
        /// How far the run stops short at its way-in end and its way-out
        /// end (m) — room for the loop that leads into it, past the
        /// clearance of the neighbouring links.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        inset_m: Option<[f64; 2]>,
    },
}

/// A body a route is dressed along: an obstacle or a robot link (a frame
/// has no surface).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum Body {
    Obstacle { name: String },
    Link { robot: String, link: String },
}

/// The article a route's auto clamps (`Via::Along`) are counted as.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct ClampArticle {
    pub manufacturer: Option<String>,
    pub model: Option<String>,
    /// A catalog reference, `id` or `id@revision`.
    pub catalog: Option<String>,
    /// Material held straight in each clamp (m).
    pub length_m: f64,
    /// The saddle's height: how far the clamp holds the line's underside
    /// off the surface (m); 5 mm when unset.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub standoff_m: Option<f64>,
    /// What the clamps leave the line free to do; rigid when unset.
    pub kind: HolderKind,
}

/// Clamp pitch of a dressed run when the route states none (m).
pub const DEFAULT_CLAMP_PITCH_M: f64 = 0.3;
/// A clamp's standoff when the article states none (m).
pub const DEFAULT_STANDOFF_M: f64 = 0.005;
/// How close to a joint's axis, and along it, a holder counts as sitting on
/// the joint housing (m).
pub const JOINT_ZONE_M: f64 = 0.07;
/// The turn either way a joint between two holders is given room for,
/// about the line's own axis (degrees): a cobot's tool line is dressed to
/// let each wrist joint turn ±180°.
pub const TWIST_ROOM_DEG: f64 = 180.0;
/// How far an auto clamp keeps from the edge of the face it sits on (m).
pub const CLAMP_EDGE_INSET_M: f64 = 0.05;
/// Line beyond what an anchor holds straight, and two radii, that the
/// body under it is not judged on: the clip itself (m).
pub const HELD_NEAR_M: f64 = 0.01;
/// Robot link parts farther from a run of line than this, by their
/// bounds, are not measured: a finding needs `CLEARANCE_MARGIN_M` (m).
const LINK_PRUNE_M: f64 = 0.1;

/// One end of a span, as the checks see the body it is held on.
#[derive(Clone, Debug, Default)]
struct HeldEnd {
    /// The rigid link group the end is held on (empty: an obstacle, a frame).
    group: Vec<(usize, usize)>,
    /// The group's shape holds the held point, within the line's radius —
    /// the line's mount: not collided with in a replay, not judged.
    holds: bool,
    /// Line near the end the group is not judged on: what the anchor holds
    /// straight, two radii, `HELD_NEAR_M`.
    near_m: f64,
}

/// A run shorter than this between two corners is a jog, not a bend: a
/// gland a millimetre off a duct's centerline is drawn through (m).
pub const MICRO_JOG_M: f64 = 0.002;

/// What a holder leaves the line free to do. A rigid clamp or bracket
/// holds everything (`Fixed`); a rotating holder holds the line's place
/// and direction and lets it roll about its own axis (`Rotating`); a guide
/// lets it slide through and roll (`Sliding`); a strap holds the point and
/// lets the line turn and roll (`Strap`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum HolderKind {
    #[default]
    Fixed,
    Rotating,
    Sliding,
    Strap,
    /// A carriage on a rail that holds the line as a fixed holder does and
    /// is pulled toward the rail's start — a retraction unit, which takes
    /// up the line as the arm folds and pays it out as it reaches.
    Retracting,
}

impl HolderKind {
    /// The line can roll about its own axis in the holder.
    pub fn rolls(self) -> bool {
        matches!(self, Self::Rotating | Self::Sliding | Self::Strap)
    }
    /// The line slides through the holder: the spans either side of it
    /// share their material.
    pub fn slides(self) -> bool {
        matches!(self, Self::Sliding)
    }
    /// The holder holds the line's direction, not only its place.
    pub fn holds_direction(self) -> bool {
        matches!(self, Self::Fixed | Self::Rotating | Self::Retracting)
    }
    /// The holder is a carriage on a rail.
    pub fn retracts(self) -> bool {
        matches!(self, Self::Retracting)
    }
    pub fn name(self) -> &'static str {
        match self {
            Self::Fixed => "fixed",
            Self::Rotating => "rotating",
            Self::Sliding => "sliding",
            Self::Strap => "strap",
            Self::Retracting => "retracting",
        }
    }
}

/// A retracting holder's rail, as declared: the carriage runs from the
/// holder's place along `axis` (the body's frame, unit) for `stroke_m`,
/// pulled back toward the start with `force_n`.
fn validate_rail(
    kind: HolderKind,
    axis: &Option<[f64; 3]>,
    stroke_m: &Option<f64>,
    force_n: &Option<f64>,
    what: &str,
) -> Result<(), String> {
    if kind.retracts() {
        if !axis.is_some_and(|a| finite_direction(&a)) {
            return Err(format!(
                "{what}: a retracting holder needs a finite, nonzero axis"
            ));
        }
        if !stroke_m.is_some_and(finite_nonneg) || !force_n.is_some_and(finite_nonneg) {
            return Err(format!(
                "{what}: a retracting holder needs a finite, nonnegative stroke_m and force_n"
            ));
        }
    } else if axis.is_some_and(|a| !finite_direction(&a))
        || stroke_m.is_some_and(|v| !finite_nonneg(v))
        || force_n.is_some_and(|v| !finite_nonneg(v))
    {
        return Err(format!("{what}: axis, stroke_m and force_n must be finite"));
    }
    Ok(())
}

/// A retracting holder's rail in the world at the current pose: where the
/// carriage runs and how it is pulled.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Rail {
    /// The rail's axis from the holder's place (world, unit).
    pub axis: [f64; 3],
    pub stroke_m: f64,
    pub force_n: f64,
}

/// Where a carrier is fixed, and how.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Holder {
    pub at: Attach,
    #[serde(default)]
    pub kind: HolderKind,
    /// The way the hose runs through the holder, in the body's frame
    /// (unit); the bisector of the legs when unset.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub direction: Option<[f64; 3]>,
    /// A retracting holder's rail: the carriage runs from the holder's
    /// place along `axis` (the body's frame, unit) for `stroke_m`, pulled
    /// back toward the start with `force_n`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis: Option<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stroke_m: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub force_n: Option<f64>,
}

/// What holds a route: a clamp (a point on a resident — on a robot link it
/// is where a moving span starts or ends) or a duct / tray (a centerline
/// the route follows end to end, with an inner cross-section).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum Support {
    Clamp {
        name: String,
        at: Attach,
        /// Material held straight in the clamp (m).
        #[serde(default)]
        length_m: f64,
        /// What the clamp leaves the line free to do; rigid when unset
        /// (`hold`: the variant's tag is `kind`).
        #[serde(default)]
        hold: HolderKind,
        /// The way the line runs through the clamp, in the body's frame
        /// (unit); the bisector of the legs when unset.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        direction: Option<[f64; 3]>,
        /// A retracting clamp's rail (see `Holder`).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        axis: Option<[f64; 3]>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        stroke_m: Option<f64>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        force_n: Option<f64>,
        #[serde(default)]
        manufacturer: Option<String>,
        #[serde(default)]
        model: Option<String>,
        #[serde(default)]
        catalog: Option<String>,
    },
    Duct {
        name: String,
        /// The centerline, world metres, at least two points.
        path: Vec<[f64; 3]>,
        /// Inner width and height (m).
        inner_m: [f64; 2],
        #[serde(default)]
        manufacturer: Option<String>,
        #[serde(default)]
        model: Option<String>,
        #[serde(default)]
        catalog: Option<String>,
    },
    /// A protective hose or 3D energy chain dressed along an arm: the
    /// holders it is fixed in, in order, and the bore the lines inside it
    /// share. Routes pass it as one way point and become a bundle.
    Carrier {
        name: String,
        holders: Vec<Holder>,
        /// The hose's own mass per metre (kg/m), for the replay of the
        /// bundle as one hose; the lines inside add theirs.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        mass_kg_m: Option<f64>,
        /// The stroke the kit's retraction takes up (m), when it has one.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        stroke_m: Option<f64>,
        /// Inner diameter (m) — what the bundle's cross-section is held against.
        /// The bore the lines share (m); a kit nobody has measured leaves it
        /// unstated, and the bundle's fill unchecked.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        inner_m: Option<f64>,
        #[serde(default)]
        od_m: Option<f64>,
        #[serde(default)]
        bend_r_moving_m: Option<f64>,
        #[serde(default)]
        torsion_deg_m: Option<f64>,
        #[serde(default)]
        manufacturer: Option<String>,
        #[serde(default)]
        model: Option<String>,
        #[serde(default)]
        catalog: Option<String>,
    },
}

impl Support {
    pub fn name(&self) -> &str {
        match self {
            Support::Clamp { name, .. }
            | Support::Duct { name, .. }
            | Support::Carrier { name, .. } => name,
        }
    }

    /// The name of a carrier's `k`-th holder as a route anchor.
    pub fn holder_name(carrier: &str, k: usize) -> String {
        format!("{carrier}/h{k}")
    }

    /// `(carrier, k)` when `name` is a carrier holder's anchor name.
    fn holder_of(name: &str) -> Option<(&str, usize)> {
        let (carrier, k) = name.rsplit_once("/h")?;
        Some((carrier, k.parse().ok()?))
    }

    fn identity(&self) -> (Option<&str>, Option<&str>, Option<&str>) {
        match self {
            Support::Clamp {
                manufacturer,
                model,
                catalog,
                ..
            }
            | Support::Duct {
                manufacturer,
                model,
                catalog,
                ..
            }
            | Support::Carrier {
                manufacturer,
                model,
                catalog,
                ..
            } => (
                manufacturer.as_deref(),
                model.as_deref(),
                catalog.as_deref(),
            ),
        }
    }
}

/// One physical line between two ports, carrying one or more connections
/// (a multi-core cable carries several), made of one spec, passing its way
/// points in order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Route {
    pub name: String,
    /// Connection names; the first decides the two end ports.
    #[serde(default)]
    pub carries: Vec<String>,
    pub spec: String,
    #[serde(default)]
    pub via: Vec<Via>,
    /// Insert axis-aligned elbows between way points (z, then x, then y).
    #[serde(default)]
    pub orthogonal: bool,
    /// Extra material beyond the measured run (a service loop), shared by
    /// the route's moving spans — or given per moving span in `slacks_m`.
    #[serde(default)]
    pub slack_m: f64,
    /// Slack per moving span, in route order; when set it replaces the
    /// shared `slack_m` (and the cut length counts its sum).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub slacks_m: Vec<f64>,
    /// Whether fixed spans are colliders for the planner, the tick
    /// verification and the clearance readout (the static clash check
    /// always runs).
    #[serde(default = "default_true")]
    pub collide: bool,
    /// Display colour, linear RGB; overrides the spec's.
    #[serde(default)]
    pub color: Option<[f32; 3]>,
    /// Clamp pitch of the runs dressed along a body (`Via::Along`), m;
    /// [`DEFAULT_CLAMP_PITCH_M`] when unset.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clamp_pitch_m: Option<f64>,
    /// The article the auto clamps are (the BOM line they make).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clamp: Option<ClampArticle>,
}

/// Everything authored about routing: the specs, the supports and the
/// routes. Rides the project; references are checked by the report, not
/// here, so a deleted resident leaves a named finding rather than a hole.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct RoutingPlan {
    pub specs: Vec<CableSpec>,
    pub supports: Vec<Support>,
    pub routes: Vec<Route>,
}

/// A finite, nonzero direction vector.
fn finite_direction(d: &[f64; 3]) -> bool {
    d.iter().all(|v| v.is_finite()) && d.iter().map(|v| v * v).sum::<f64>() > 1e-12
}

/// The angle of a rotation with its turn about each of `axes` taken out
/// (the swing of a swing–twist split, applied axis by axis).
fn swing_angle(q: &nalgebra::UnitQuaternion<f64>, axes: &[Vector3<f64>]) -> f64 {
    let mut q = *q;
    for u in axes {
        if u.norm() < 1e-9 {
            continue;
        }
        let u = u.normalize();
        let p = q.imag().dot(&u) * u;
        let twist = nalgebra::Quaternion::from_parts(q.w, p);
        if twist.norm() < 1e-9 {
            continue;
        }
        let twist = nalgebra::UnitQuaternion::from_quaternion(twist);
        q *= twist.inverse();
    }
    q.angle()
}

/// A holder's direction in the world, from the body's frame.
fn world_direction(pose: &Isometry3<f64>, d: &Option<[f64; 3]>) -> Option<Vector3<f64>> {
    let v = pose.rotation * Vector3::from((*d)?);
    (v.norm() > 1e-9).then(|| v.normalize())
}

fn finite_nonneg(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

fn finite_pos(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn finite_point(p: &[f64; 3]) -> bool {
    p.iter().all(|v| v.is_finite())
}

impl Route {
    /// All the slack the route carries.
    pub fn slack_total_m(&self) -> f64 {
        if self.slacks_m.is_empty() {
            self.slack_m
        } else {
            self.slacks_m.iter().sum()
        }
    }

    /// The slack of the `k`-th moving span out of `moving` — its own figure
    /// when given per span, otherwise an equal share of `slack_m`.
    pub fn slack_of_moving(&self, k: usize, moving: usize) -> f64 {
        match self.slacks_m.get(k) {
            Some(v) => *v,
            None if !self.slacks_m.is_empty() => 0.0,
            None if moving > 0 => self.slack_m / moving as f64,
            None => 0.0,
        }
    }
}

impl RoutingPlan {
    pub fn is_empty(&self) -> bool {
        self.specs.is_empty() && self.supports.is_empty() && self.routes.is_empty()
    }

    pub fn spec(&self, name: &str) -> Option<&CableSpec> {
        self.specs.iter().find(|s| s.name == name)
    }

    pub fn support(&self, name: &str) -> Option<&Support> {
        self.supports.iter().find(|s| s.name() == name)
    }

    pub fn route(&self, name: &str) -> Option<&Route> {
        self.routes.iter().find(|r| r.name == name)
    }

    /// Validate representation, not design: names and numbers. Broken
    /// references survive so the report can name them.
    pub fn validate(&self) -> Result<(), String> {
        let mut names = BTreeSet::new();
        for spec in &self.specs {
            if spec.name.trim().is_empty() || !names.insert(spec.name.as_str()) {
                return Err(format!("invalid or duplicate cable spec: {:?}", spec.name));
            }
            for (key, value) in [
                ("od_m", spec.od_m),
                ("bend_r_fixed_m", spec.bend_r_fixed_m),
                ("bend_r_moving_m", spec.bend_r_moving_m),
                ("mass_kg_m", spec.mass_kg_m),
                ("max_length_m", spec.max_length_m),
                ("torsion_deg_m", spec.torsion_deg_m),
                ("order_step_m", spec.order_step_m),
            ] {
                if value.is_some_and(|v| !finite_pos(v)) {
                    return Err(format!("{}: {key} must be finite and positive", spec.name));
                }
            }
            if !finite_nonneg(spec.end_allowance_m) {
                return Err(format!(
                    "{}: end_allowance_m must be finite and nonnegative",
                    spec.name
                ));
            }
            if spec.sold_lengths_m.iter().any(|v| !finite_pos(*v)) {
                return Err(format!(
                    "{}: sold lengths must be finite and positive",
                    spec.name
                ));
            }
            if spec
                .color
                .is_some_and(|c| c.iter().any(|v| !v.is_finite() || *v < 0.0))
            {
                return Err(format!(
                    "{}: color must be nonnegative linear RGB",
                    spec.name
                ));
            }
        }
        names.clear();
        for support in &self.supports {
            if support.name().trim().is_empty() || !names.insert(support.name()) {
                return Err(format!(
                    "invalid or duplicate support: {:?}",
                    support.name()
                ));
            }
            match support {
                Support::Clamp {
                    at,
                    length_m,
                    hold,
                    direction,
                    axis,
                    stroke_m,
                    force_n,
                    ..
                } => {
                    at.validate()
                        .map_err(|e| format!("{}: {e}", support.name()))?;
                    if !finite_nonneg(*length_m) {
                        return Err(format!(
                            "{}: length_m must be finite and nonnegative",
                            support.name()
                        ));
                    }
                    if direction.is_some_and(|d| !finite_direction(&d)) {
                        return Err(format!(
                            "{}: direction must be a finite, nonzero vector",
                            support.name()
                        ));
                    }
                    validate_rail(*hold, axis, stroke_m, force_n, support.name())?;
                }
                Support::Duct { path, inner_m, .. } => {
                    if path.len() < 2 || path.iter().any(|p| !finite_point(p)) {
                        return Err(format!(
                            "{}: a duct path needs two or more finite points",
                            support.name()
                        ));
                    }
                    if inner_m.iter().any(|v| !finite_pos(*v)) {
                        return Err(format!(
                            "{}: inner_m must be finite and positive",
                            support.name()
                        ));
                    }
                }
                Support::Carrier {
                    holders,
                    inner_m,
                    od_m,
                    bend_r_moving_m,
                    torsion_deg_m,
                    mass_kg_m,
                    stroke_m,
                    ..
                } => {
                    if holders.is_empty() {
                        return Err(format!(
                            "{}: a carrier has at least one holder",
                            support.name()
                        ));
                    }
                    for (k, h) in holders.iter().enumerate() {
                        h.at.validate()
                            .map_err(|e| format!("{} holder {k}: {e}", support.name()))?;
                        if h.direction.is_some_and(|d| !finite_direction(&d)) {
                            return Err(format!(
                                "{} holder {k}: direction must be a finite, nonzero vector",
                                support.name()
                            ));
                        }
                        validate_rail(
                            h.kind,
                            &h.axis,
                            &h.stroke_m,
                            &h.force_n,
                            &format!("{} holder {k}", support.name()),
                        )?;
                    }
                    if [inner_m, od_m, bend_r_moving_m, torsion_deg_m, mass_kg_m]
                        .iter()
                        .any(|v| v.is_some_and(|v| !finite_pos(v)))
                        || stroke_m.is_some_and(|v| !finite_nonneg(v))
                    {
                        return Err(format!(
                            "{}: inner_m, od_m, bend_r_moving_m, torsion_deg_m and mass_kg_m must be finite and positive, stroke_m nonnegative",
                            support.name()
                        ));
                    }
                }
            }
        }
        names.clear();
        for route in &self.routes {
            if route.name.trim().is_empty() || !names.insert(route.name.as_str()) {
                return Err(format!("invalid or duplicate route: {:?}", route.name));
            }
            if route.spec.trim().is_empty() {
                return Err(format!("{}: a route names its spec", route.name));
            }
            if !finite_nonneg(route.slack_m) || route.slacks_m.iter().any(|v| !finite_nonneg(*v)) {
                return Err(format!(
                    "{}: slack_m must be finite and nonnegative",
                    route.name
                ));
            }
            for via in &route.via {
                match via {
                    Via::Point { position } if !finite_point(position) => {
                        return Err(format!("{}: way points must be finite", route.name));
                    }
                    Via::Support { name } if name.trim().is_empty() => {
                        return Err(format!(
                            "{}: a support way point names its support",
                            route.name
                        ));
                    }
                    Via::Along {
                        on,
                        pitch_m,
                        face,
                        offset_m,
                        inset_m,
                    } => {
                        if offset_m.is_some_and(|v| !v.is_finite()) {
                            return Err(format!("{}: offset_m must be finite", route.name));
                        }
                        if inset_m.is_some_and(|v| v.iter().any(|x| !x.is_finite() || *x < 0.0)) {
                            return Err(format!(
                                "{}: inset_m must be finite and nonnegative",
                                route.name
                            ));
                        }
                        if face.as_deref().is_some_and(|f| face_axis(f).is_none()) {
                            return Err(format!(
                                "{}: an along face is one of +x, -x, +y, -y, +z, -z",
                                route.name
                            ));
                        }
                        let named = match on {
                            Body::Obstacle { name } => !name.trim().is_empty(),
                            Body::Link { robot, link } => {
                                !robot.trim().is_empty() && !link.trim().is_empty()
                            }
                        };
                        if !named {
                            return Err(format!(
                                "{}: an along way point names its body",
                                route.name
                            ));
                        }
                        if pitch_m.is_some_and(|v| !finite_pos(v)) {
                            return Err(format!(
                                "{}: pitch_m must be finite and positive",
                                route.name
                            ));
                        }
                    }
                    _ => {}
                }
            }
            if route.clamp_pitch_m.is_some_and(|v| !finite_pos(v)) {
                return Err(format!(
                    "{}: clamp_pitch_m must be finite and positive",
                    route.name
                ));
            }
            if let Some(clamp) = &route.clamp {
                if !finite_nonneg(clamp.length_m)
                    || clamp.standoff_m.is_some_and(|v| !finite_nonneg(v))
                {
                    return Err(format!(
                        "{}: clamp length_m and standoff_m must be finite and nonnegative",
                        route.name
                    ));
                }
            }
            if route
                .color
                .is_some_and(|c| c.iter().any(|v| !v.is_finite() || *v < 0.0))
            {
                return Err(format!(
                    "{}: color must be nonnegative linear RGB",
                    route.name
                ));
            }
        }
        Ok(())
    }
}

// ------------------------------------------------------------------ derived

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SpanKind {
    /// Both ends on static residents: a closed-form polyline, checked here.
    Fixed,
    /// An end on a robot link (or a carried object): drawn at time zero,
    /// verified by the rope replay.
    Moving,
    /// Both ends on one moving body (two clamps on a forearm): it rides
    /// the body — no free length to size, no twist — and the replay only
    /// watches what it touches.
    Dressed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AnchorKind {
    Port,
    Clamp,
}

/// A point the route is held at, with its arc length along the route.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RouteAnchor {
    pub name: String,
    pub kind: AnchorKind,
    pub position: [f64; 3],
    /// A port's exit direction (world, unit); clamps have none.
    pub exit: Option<[f64; 3]>,
    /// The way the line runs through the holder (world, unit, the route's
    /// direction of travel): a port's stub, an auto clamp's run, and at a
    /// declared holder — its orientation is not modelled — the bisector of
    /// the two legs. What a replay lays the held material along.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tangent: Option<[f64; 3]>,
    pub moving: bool,
    pub s_m: f64,
    /// What holds it, when known: the port's place, the clamp's, a
    /// carrier's holder, or the body an auto clamp was laid on.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attach: Option<Attach>,
    /// Generated by a `Via::Along` run rather than declared.
    #[serde(default)]
    pub auto: bool,
    /// What the holder leaves the line free to do (a port: fixed, by its
    /// stub).
    #[serde(default)]
    pub holder: HolderKind,
    /// A retracting holder's rail (world, at the current pose).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rail: Option<Rail>,
}

/// A stretch between two anchors.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Span {
    pub kind: SpanKind,
    pub from: String,
    pub to: String,
    pub from_s: f64,
    pub to_s: f64,
    /// The line slides through the holder this span starts at: the span
    /// and the one before it share their material and are replayed as one.
    #[serde(default)]
    pub slides: bool,
}

/// A corner of the polyline and the fillet it got.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Bend {
    /// The sharp corner the fillet replaces.
    pub position: [f64; 3],
    pub s_m: f64,
    pub angle_deg: f64,
    /// The radius drawn; `None` when the spec states no bend radius (a
    /// sharp corner is drawn).
    pub radius_m: Option<f64>,
    pub required_m: Option<f64>,
    /// `Some(false)`: the required radius does not fit between the
    /// neighbouring corners; `None` without a requirement.
    pub ok: Option<bool>,
}

/// One end of a route: the port and the resident it sits on.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Endpoint {
    pub port: String,
    pub target: String,
    pub target_kind: PartTargetKind,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub terminal: Option<String>,
}

/// The derived shape of one route at the scene's current poses.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RouteGeometry {
    pub name: String,
    pub spec: String,
    pub kind: LineKind,
    pub medium: Option<Medium>,
    pub carries: Vec<String>,
    pub from: Endpoint,
    pub to: Endpoint,
    /// World points, bends tessellated.
    pub centerline: Vec<[f64; 3]>,
    /// Arc length at each centerline point.
    pub s: Vec<f64>,
    pub anchors: Vec<RouteAnchor>,
    pub spans: Vec<Span>,
    pub bends: Vec<Bend>,
    /// Ducts passed, clamps held at, carriers dressed in (in order).
    pub ducts: Vec<String>,
    pub clamps: Vec<String>,
    pub carriers: Vec<String>,
    /// The clamps a `Via::Along` run generated (`<route>/c<k>`), in order.
    pub auto_clamps: Vec<String>,
    /// The measured run.
    pub length_m: f64,
    /// How much of the run lies inside a duct (m), before the fillets.
    pub ducted_m: f64,
    /// Run + end allowances + slack.
    pub cut_length_m: f64,
    /// The shortest sold length that fits, or the cut length rounded up to
    /// the order step; `None` when the spec states neither.
    pub purchase_length_m: Option<f64>,
    pub od_m: Option<f64>,
    pub color: [f32; 3],
}

impl RouteGeometry {
    /// The centerline point at arc length `s` (clamped to the ends).
    pub fn point_at(&self, s: f64) -> Option<[f64; 3]> {
        if self.centerline.is_empty() {
            return None;
        }
        let k = self.s.partition_point(|v| *v <= s).saturating_sub(1);
        let j = (k + 1).min(self.centerline.len() - 1);
        if k == j || self.s[j] <= self.s[k] {
            return Some(self.centerline[k]);
        }
        let u = ((s - self.s[k]) / (self.s[j] - self.s[k])).clamp(0.0, 1.0);
        let a = self.centerline[k];
        let b = self.centerline[j];
        Some(std::array::from_fn(|i| a[i] + (b[i] - a[i]) * u))
    }

    /// The centerline points of one span, inclusive of both ends.
    pub fn span_points(&self, span: &Span) -> Vec<[f64; 3]> {
        self.centerline
            .iter()
            .zip(&self.s)
            .filter(|(_, s)| **s >= span.from_s - 1e-9 && **s <= span.to_s + 1e-9)
            .map(|(p, _)| *p)
            .collect()
    }

    pub fn fixed_length_m(&self) -> f64 {
        self.spans
            .iter()
            .filter(|s| s.kind == SpanKind::Fixed)
            .map(|s| s.to_s - s.from_s)
            .sum()
    }

    pub fn moving_length_m(&self) -> f64 {
        self.spans
            .iter()
            .filter(|s| s.kind == SpanKind::Moving)
            .map(|s| s.to_s - s.from_s)
            .sum()
    }
}

/// The index of the sample of `points` nearest `p`.
fn nearest_sample(points: &[[f64; 3]], p: [f64; 3]) -> usize {
    let p = Vector3::from(p);
    points
        .iter()
        .enumerate()
        .map(|(i, q)| (i, (Vector3::from(*q) - p).norm_squared()))
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0)
}

/// Where a check puts its findings: severity, code, message, place, and
/// the way along the route.
type FindingSink<'a> = dyn FnMut(&str, &str, String, Option<[f64; 3]>, Option<f64>) + 'a;

/// The links a span's replay collides with, and the ones it is exempt
/// from, as `(robot, link)` pairs.
type ReplayLinks = (Vec<(String, String)>, Vec<(String, String)>);

/// Clamp lines of the BOM grouped by article (manufacturer, model, kind):
/// the routes and clamp names under each.
type ClampGroups =
    BTreeMap<(Option<String>, Option<String>, Option<String>), Vec<(String, String)>>;

/// One check result, located on the route when it has a place.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RoutingFinding {
    /// `error` | `warning` | `info`.
    pub severity: String,
    pub code: String,
    /// The route (or, for `route_unrouted`, the connection).
    pub route: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub s_m: Option<f64>,
    /// When, on a baked cycle (the static checks have none).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub t_s: Option<f64>,
    /// The replayed rope the finding was read off, and the sample on it:
    /// the place rides the line as the cycle plays.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rope: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sample: Option<usize>,
}

/// One line of the cable list.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RouteRow {
    pub name: String,
    pub spec: String,
    pub kind: LineKind,
    pub medium: Option<Medium>,
    pub carries: Vec<String>,
    pub from: Option<Endpoint>,
    pub to: Option<Endpoint>,
    pub manufacturer: Option<String>,
    pub model: Option<String>,
    pub catalog: Option<String>,
    pub od_mm: Option<f64>,
    pub length_m: Option<f64>,
    pub fixed_m: Option<f64>,
    pub moving_m: Option<f64>,
    pub cut_length_m: Option<f64>,
    pub purchase_length_m: Option<f64>,
    pub ducts: Vec<String>,
    pub clamps: Vec<String>,
    pub carriers: Vec<String>,
    /// Clamps the route's dressed runs generated.
    pub auto_clamps: usize,
    pub bends: usize,
    /// `ok` | `warning` | `error`: the worst finding on this route.
    pub status: String,
    pub findings: usize,
}

/// The cable list and the static checks, over every route in the plan.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RoutingReport {
    pub routes: Vec<RouteRow>,
    pub findings: Vec<RoutingFinding>,
    /// The resolved geometry per route (same order as `routes`); `None`
    /// where the route could not be resolved.
    #[serde(skip)]
    pub geometries: Vec<Option<RouteGeometry>>,
}

impl RoutingReport {
    pub fn errors(&self) -> impl Iterator<Item = &RoutingFinding> {
        self.findings.iter().filter(|f| f.severity == "error")
    }
}

/// Clearance under which a fixed span is reported close to something.
pub const CLEARANCE_MARGIN_M: f64 = 0.02;
/// How full a duct or a carrier may be, by cross-section, before the
/// bundle is reported (the common cable-tray rule of thumb).
pub const FILL_LIMIT: f64 = 0.5;
/// Angle (degrees) a line may leave a connector off its exit direction.
pub const EXIT_TOLERANCE_DEG: f64 = 60.0;
/// Arc step of the fillet tessellation.
const ARC_STEP: f64 = PI / 36.0;
const EPS: f64 = 1e-9;

fn medium_color(medium: Option<Medium>) -> [f32; 3] {
    match medium {
        Some(Medium::Power) => [0.08, 0.08, 0.09],
        Some(Medium::Signal) => [0.12, 0.33, 0.75],
        Some(Medium::Network) => [0.12, 0.55, 0.22],
        Some(Medium::Pneumatic) => [0.38, 0.62, 0.86],
        None => [0.45, 0.45, 0.47],
    }
}

fn medium_name(medium: Medium) -> &'static str {
    match medium {
        Medium::Power => "power",
        Medium::Pneumatic => "pneumatic",
        Medium::Signal => "signal",
        Medium::Network => "network",
    }
}

fn to_array(p: Point3<f64>) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn round_m(value: f64) -> f64 {
    // `+ 0.0` folds a negative zero into zero, so an empty sum prints as 0.
    (value * 1000.0).round() / 1000.0 + 0.0
}

/// What a vertex is held by, when it is an anchor.
#[derive(Clone)]
struct VertAnchor {
    name: String,
    kind: AnchorKind,
    moving: bool,
    exit: Option<Vector3<f64>>,
    attach: Option<Attach>,
    auto: bool,
    holder: HolderKind,
    /// The way the line runs through the holder, as declared (world).
    direction: Option<Vector3<f64>>,
    rail: Option<RailWorld>,
}

/// A retracting holder's rail in the world.
#[derive(Debug, Clone, Copy, PartialEq)]
struct RailWorld {
    axis: Vector3<f64>,
    stroke: f64,
    force: f64,
}

/// The rail of a retracting holder in the world, from the body's frame.
fn rail_world(
    pose: &Isometry3<f64>,
    kind: HolderKind,
    axis: &Option<[f64; 3]>,
    stroke_m: &Option<f64>,
    force_n: &Option<f64>,
) -> Option<RailWorld> {
    if !kind.retracts() {
        return None;
    }
    Some(RailWorld {
        axis: world_direction(pose, axis)?,
        stroke: (*stroke_m)?,
        force: (*force_n)?,
    })
}

struct Vert {
    p: Point3<f64>,
    anchor: Option<VertAnchor>,
    fillet: bool,
    /// The segment from this vertex to the next is laid as it is — inside
    /// a duct, or along a body between two auto clamps — and gets no
    /// orthogonal elbows.
    in_duct: bool,
    /// Line next to this vertex that must stay straight (a connector's
    /// stub, half a clamp's holding length): a fillet at the neighbouring
    /// vertex may not eat into it.
    reserve: f64,
}

impl Scene {
    pub fn routing_plan(&self) -> &RoutingPlan {
        &self.routing_plan
    }

    /// Two attachments on one rigid body: the same obstacle, or links of
    /// one robot joined by fixed joints.
    fn same_body(&self, a: Option<&Attach>, b: Option<&Attach>) -> bool {
        match (a, b) {
            (Some(Attach::Obstacle { name: x, .. }), Some(Attach::Obstacle { name: y, .. })) => {
                x == y
            }
            (
                Some(Attach::Link {
                    robot: ra,
                    link: la,
                    ..
                }),
                Some(Attach::Link {
                    robot: rb,
                    link: lb,
                    ..
                }),
            ) if ra == rb => self.robot_index(ra).is_some_and(|r| {
                let model = &self.robots()[r].model;
                match (model.link_index(la), model.link_index(lb)) {
                    (Some(i), Some(j)) => rigid_group(model, i).contains(&j),
                    _ => false,
                }
            }),
            _ => false,
        }
    }

    /// A body's origin in the world: where the way continues to when the
    /// next way point is another body.
    fn body_origin(&self, on: &Body) -> Option<Point3<f64>> {
        let at = match on {
            Body::Obstacle { name } => Attach::Obstacle {
                name: name.clone(),
                offset_m: [0.0; 3],
            },
            Body::Link { robot, link } => Attach::Link {
                robot: robot.clone(),
                link: link.clone(),
                offset_m: [0.0; 3],
            },
        };
        self.attach_point(&at).ok().map(|(p, _, _)| p)
    }

    /// The clamps of a run dressed along `on`, as attachments in the body's
    /// frame: on a face of its collision bounds — `face` when given, else
    /// the side face (one along the body's long axis: a run lies along an
    /// arm link or a column, not across its end) nearest the way in
    /// (`prev`) and the way out (`next`) — `off` above the surface, every
    /// `pitch` from the entry's projection to the exit's. A lateral offset
    /// smaller than a pitch is taken up by the way in, so the run itself is
    /// parallel to the body's edge; `offset` moves it sideways on the face.
    /// On a robot link the run's ends are drawn in until they clear the
    /// robot's other links (the joint housings at either end), and
    /// `room` draws them in further — the loop that leads in needs it.
    #[allow(clippy::too_many_arguments)]
    fn dress_run(
        &self,
        on: &Body,
        prev: Point3<f64>,
        next: Point3<f64>,
        off: f64,
        pitch: f64,
        face: Option<&str>,
        offset: f64,
        room: [f64; 2],
    ) -> Result<Vec<Attach>, String> {
        use parry3d_f64::bounding_volume::BoundingVolume;
        let mut link_of: Option<(usize, usize)> = None;
        let (pose, lo, hi) = match on {
            Body::Obstacle { name } => {
                let (obstacle, collider) = self
                    .obstacle_with_collider(name)
                    .ok_or_else(|| format!("unknown obstacle `{name}`"))?;
                let (lo, hi) = collider
                    .aabb(&Isometry3::identity())
                    .ok_or_else(|| format!("`{name}` has no collision geometry to dress along"))?;
                (obstacle.pose, lo, hi)
            }
            Body::Link { robot, link } => {
                let r = self
                    .robot_index(robot)
                    .ok_or_else(|| format!("unknown robot `{robot}`"))?;
                let entry = &self.robots()[r];
                let l = entry
                    .model
                    .link_index(link)
                    .ok_or_else(|| format!("robot `{robot}` has no link `{link}`"))?;
                let mut merged: Option<parry3d_f64::bounding_volume::Aabb> = None;
                for (part_pose, shape) in entry.collider.link_parts(l) {
                    let b = shape.compute_aabb(part_pose);
                    merged = Some(match merged {
                        Some(acc) => acc.merged(&b),
                        None => b,
                    });
                }
                let b = merged.ok_or_else(|| {
                    format!("link `{robot}/{link}` has no collision geometry to dress along")
                })?;
                link_of = Some((r, l));
                (
                    self.link_poses_for(r)[l],
                    [b.mins.x, b.mins.y, b.mins.z],
                    [b.maxs.x, b.maxs.y, b.maxs.z],
                )
            }
        };
        let inv = pose.inverse();
        let (qp, qn) = (inv * prev, inv * next);
        // Clamps keep off the edges of a face — a screw's worth, 5 cm, or
        // what a thin body has — so a run up a column starts clear of the
        // plate it stands on.
        let inset = |k: usize| ((hi[k] - lo[k]) / 2.0).min(CLAMP_EDGE_INSET_M);
        let project = |q: Point3<f64>, k: usize, side: f64| -> Point3<f64> {
            let mut p = q;
            for j in 0..3 {
                if j != k {
                    p[j] = q[j].clamp(lo[j] + inset(j), hi[j] - inset(j));
                }
            }
            p[k] = if side > 0.0 { hi[k] + off } else { lo[k] - off };
            p
        };
        // The faces a run may lie on: the one asked for, or the faces along
        // the long axis of a long body (all six of a squat one).
        let extent: Vec<f64> = (0..3).map(|k| hi[k] - lo[k]).collect();
        let long = (0..3)
            .max_by(|a, b| {
                extent[*a]
                    .partial_cmp(&extent[*b])
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .expect("three axes");
        let elongated = (0..3)
            .filter(|k| *k != long)
            .all(|k| extent[long] >= 1.5 * extent[k]);
        let faces: Vec<(usize, f64)> = match face.and_then(face_axis) {
            Some(f) => vec![f],
            None => (0..3)
                .filter(|k| !elongated || *k != long)
                .flat_map(|k| [(k, -1.0), (k, 1.0)])
                .collect(),
        };
        let mut best: Option<(f64, Point3<f64>, Point3<f64>, usize)> = None;
        for (k, side) in faces {
            let (e, x) = (project(qp, k, side), project(qn, k, side));
            let cost = (qp - e).norm() + (qn - x).norm();
            if best.is_none_or(|b| cost < b.0) {
                best = Some((cost, e, x, k));
            }
        }
        let (_, mut e, mut x, k) = best.expect("at least one face");
        let lateral: Vec<usize> = (0..3).filter(|j| *j != k).collect();
        let d = x - e;
        let lat = if d[lateral[0]].abs() >= d[lateral[1]].abs() {
            lateral[1]
        } else {
            lateral[0]
        };
        if d[lat].abs() < pitch {
            e[lat] = x[lat];
        }
        // Sideways on the face by `offset`, within the face.
        if offset != 0.0 {
            for p in [&mut e, &mut x] {
                p[lat] = (p[lat] + offset).clamp(lo[lat], hi[lat]);
            }
        }
        // On a robot link, the ends come in until they clear the other
        // links — a clamp in the next joint's housing holds the line in it.
        if let Some((r, l)) = link_of {
            let model = &self.robots()[r].model;
            let own = rigid_group(model, l);
            let poses = self.link_poses_for(r);
            let clear = |p: Point3<f64>| {
                let world = pose * p;
                (0..model.links.len())
                    .filter(|m| !own.contains(m))
                    .all(|m| {
                        self.link_point_distance(r, m, &poses, &world)
                            .is_none_or(|dist| dist > off)
                    })
            };
            const STEP_M: f64 = 0.005;
            for _ in 0..200 {
                let span = x - e;
                if span.norm() <= 2.0 * STEP_M {
                    break;
                }
                let dir = span / span.norm();
                let (ce, cx) = (clear(e), clear(x));
                if ce && cx {
                    break;
                }
                if !ce {
                    e += dir * STEP_M;
                }
                if !cx {
                    x -= dir * STEP_M;
                }
            }
        }
        // The asked-for room at either end, past the clearance.
        if room[0] + room[1] > 0.0 {
            let span = x - e;
            let len = span.norm();
            if room[0] + room[1] >= len - 1e-9 {
                return Err(format!(
                    "the inset ({:.3} + {:.3} m) leaves nothing of the {:.3} m run along the body",
                    room[0], room[1], len
                ));
            }
            let dir = span / len;
            e += dir * room[0];
            x -= dir * room[1];
        }
        let d = x - e;
        let len = d.norm();
        let n = if len < 1e-6 {
            0
        } else {
            (len / pitch - 1e-9).ceil().max(1.0) as usize
        };
        let place = |p: Point3<f64>| match on {
            Body::Obstacle { name } => Attach::Obstacle {
                name: name.clone(),
                offset_m: [p.x, p.y, p.z],
            },
            Body::Link { robot, link } => Attach::Link {
                robot: robot.clone(),
                link: link.clone(),
                offset_m: [p.x, p.y, p.z],
            },
        };
        Ok((0..=n)
            .map(|i| {
                let p = if n == 0 {
                    e
                } else {
                    e + d * (i as f64 / n as f64)
                };
                place(p)
            })
            .collect())
    }

    pub fn set_routing_plan(&mut self, plan: RoutingPlan) -> Result<(), String> {
        plan.validate()?;
        self.routing_plan = plan;
        self.plan_gen = self.plan_gen.wrapping_add(1);
        Ok(())
    }

    /// The world point an attachment names, the resident's pose, and
    /// whether that resident moves with a robot (a link below the root, an
    /// object a robot carries).
    pub fn attach_point(&self, at: &Attach) -> Result<(Point3<f64>, Isometry3<f64>, bool), String> {
        match at {
            Attach::Obstacle { name, offset_m } => {
                let obstacle = self
                    .obstacles()
                    .iter()
                    .find(|o| &o.name == name)
                    .ok_or_else(|| format!("unknown obstacle `{name}`"))?;
                let moving = self.is_attached(name);
                Ok((
                    obstacle.pose * Point3::from(*offset_m),
                    obstacle.pose,
                    moving,
                ))
            }
            Attach::Link {
                robot,
                link,
                offset_m,
            } => {
                let r = self
                    .robot_index(robot)
                    .ok_or_else(|| format!("unknown robot `{robot}`"))?;
                let model = &self.robots()[r].model;
                let l = model
                    .link_index(link)
                    .ok_or_else(|| format!("robot `{robot}` has no link `{link}`"))?;
                let pose = self.link_poses_for(r)[l];
                Ok((pose * Point3::from(*offset_m), pose, link_moves(model, l)))
            }
            Attach::Frame { name, offset_m } => {
                let frame = self
                    .frame(name)
                    .ok_or_else(|| format!("unknown frame `{name}`"))?;
                Ok((frame.pose * Point3::from(*offset_m), frame.pose, false))
            }
        }
    }

    /// True when some anchor of the plan rides a robot link below its root
    /// — the drawn shape then changes with the joints.
    pub fn routing_has_moving_anchors(&self) -> bool {
        let plan = &self.routing_plan;
        let link_moves = |at: &Attach| match at {
            Attach::Link { robot, link, .. } => self.robot_index(robot).is_some_and(|r| {
                let model = &self.robots()[r].model;
                model.link_index(link).is_some_and(|l| link_moves(model, l))
            }),
            Attach::Obstacle { name, .. } => self.is_attached(name),
            Attach::Frame { .. } => false,
        };
        plan.supports.iter().any(|s| match s {
            Support::Clamp { at, .. } => link_moves(at),
            Support::Carrier { holders, .. } => holders.iter().any(|h| link_moves(&h.at)),
            Support::Duct { .. } => false,
        }) || self
            .connection_plan()
            .ports
            .iter()
            .any(|p| p.geometry.as_ref().is_some_and(|g| link_moves(&g.at)))
    }

    fn port_with_geometry<'a>(
        &'a self,
        name: &str,
    ) -> Result<(&'a Port, &'a PortGeometry), String> {
        let port = self
            .connection_plan()
            .ports
            .iter()
            .find(|p| p.name == name)
            .ok_or_else(|| format!("unknown port `{name}`"))?;
        let geometry = port.geometry.as_ref().ok_or_else(|| {
            format!("port `{name}` has no geometry (where it sits and which way it exits)")
        })?;
        Ok((port, geometry))
    }

    fn connection<'a>(&'a self, name: &str) -> Result<&'a Connection, String> {
        self.connection_plan()
            .links
            .iter()
            .find(|c| c.name == name)
            .ok_or_else(|| format!("unknown connection `{name}`"))
    }

    /// The derived geometry of one route at the current poses.
    pub fn route_geometry(&self, name: &str) -> Result<RouteGeometry, String> {
        let plan = &self.routing_plan;
        let route = plan
            .route(name)
            .ok_or_else(|| format!("unknown route `{name}`"))?;
        let spec = plan
            .spec(&route.spec)
            .ok_or_else(|| format!("route `{name}` names an unknown spec `{}`", route.spec))?;
        let first = route
            .carries
            .first()
            .ok_or_else(|| format!("route `{name}` carries no connection"))?;
        let connection = self.connection(first)?;
        let (port_a, geo_a) = self.port_with_geometry(&connection.source)?;
        let (port_b, geo_b) = self.port_with_geometry(&connection.target)?;
        let medium = Some(port_a.medium);
        let endpoint = |port: &Port| Endpoint {
            port: port.name.clone(),
            target: port.target.clone(),
            target_kind: port.target_kind,
            terminal: port.terminal.clone(),
        };

        let anchor_of = |port: &Port,
                         geo: &PortGeometry|
         -> Result<(Point3<f64>, Vector3<f64>, bool), String> {
            let (p, pose, moving) = self.attach_point(&geo.at)?;
            let exit = pose.rotation * Vector3::from(geo.exit);
            let exit = exit
                .try_normalize(EPS)
                .ok_or_else(|| format!("port `{}` has a zero exit direction", port.name))?;
            Ok((p, exit, moving))
        };
        let (pa, ea, moving_a) = anchor_of(port_a, geo_a)?;
        let (pb, eb, moving_b) = anchor_of(port_b, geo_b)?;

        let r_req = spec.bend_r_fixed_m;
        // Below this, a component of a move is noise an orthogonal elbow
        // would turn into a jog shorter than the fillet: the spec's bend
        // radius, or the line's diameter, or 5 mm.
        let tol = r_req
            .unwrap_or(0.0)
            .max(spec.od_m.unwrap_or(0.0))
            .max(0.005);
        let orthogonal = route.orthogonal;
        // The axes an orthogonal move takes, in order (z, x, y), keeping
        // only the components that matter.
        let significant = |d: Vector3<f64>| -> Vec<usize> {
            [2usize, 0, 1]
                .into_iter()
                .filter(|k| d[*k].abs() > tol)
                .collect()
        };
        let axis_dir = |k: usize, d: f64| -> Vector3<f64> {
            let mut v = Vector3::zeros();
            v[k] = d.signum();
            v
        };
        let first_move = |from: Point3<f64>, to: Point3<f64>| -> Option<Vector3<f64>> {
            let d = to - from;
            match significant(d).as_slice() {
                [k, _, ..] if orthogonal => Some(axis_dir(*k, d[*k])),
                _ => d.try_normalize(EPS),
            }
        };
        let last_move = |from: Point3<f64>, to: Point3<f64>| -> Option<Vector3<f64>> {
            let d = to - from;
            let sig = significant(d);
            match sig.as_slice() {
                [_, .., k] if orthogonal => Some(axis_dir(*k, d[*k])),
                _ => d.try_normalize(EPS),
            }
        };

        // Raw way points between the two ports, in route order; ducts are
        // expanded in a second pass, once their neighbours are known.
        enum Raw {
            Point(Point3<f64>),
            Clamp {
                p: Point3<f64>,
                name: String,
                moving: bool,
                reserve: f64,
                attach: Attach,
                kind: HolderKind,
                direction: Option<Vector3<f64>>,
                rail: Option<RailWorld>,
            },
            Duct {
                name: String,
                path: Vec<Point3<f64>>,
            },
            Along {
                on: Body,
                pitch: f64,
                face: Option<String>,
                offset: f64,
                inset: [f64; 2],
            },
        }
        let mut raw: Vec<Raw> = Vec::new();
        let mut carriers: Vec<String> = Vec::new();
        for via in &route.via {
            match via {
                Via::Point { position } => raw.push(Raw::Point(Point3::from(*position))),
                Via::Along {
                    on,
                    pitch_m,
                    face,
                    offset_m,
                    inset_m,
                } => raw.push(Raw::Along {
                    on: on.clone(),
                    pitch: pitch_m
                        .or(route.clamp_pitch_m)
                        .unwrap_or(DEFAULT_CLAMP_PITCH_M),
                    face: face.clone(),
                    offset: offset_m.unwrap_or(0.0),
                    inset: inset_m.unwrap_or([0.0; 2]),
                }),
                Via::Support { name: support } => match plan.support(support) {
                    None => {
                        return Err(format!(
                            "route `{name}` passes an unknown support `{support}`"
                        ))
                    }
                    Some(Support::Clamp {
                        at,
                        length_m,
                        hold,
                        direction,
                        axis,
                        stroke_m,
                        force_n,
                        ..
                    }) => {
                        let (p, pose, moving) = self
                            .attach_point(at)
                            .map_err(|e| format!("clamp `{support}`: {e}"))?;
                        raw.push(Raw::Clamp {
                            p,
                            name: support.clone(),
                            moving,
                            reserve: length_m / 2.0,
                            attach: at.clone(),
                            kind: *hold,
                            direction: world_direction(&pose, direction),
                            rail: rail_world(&pose, *hold, axis, stroke_m, force_n),
                        });
                    }
                    Some(Support::Duct { path, .. }) => raw.push(Raw::Duct {
                        name: support.clone(),
                        path: path.iter().map(|p| Point3::from(*p)).collect(),
                    }),
                    Some(Support::Carrier { holders, .. }) => {
                        // The hose's holders, in order.
                        for (k, h) in holders.iter().enumerate() {
                            let (p, pose, moving) = self
                                .attach_point(&h.at)
                                .map_err(|e| format!("carrier `{support}` holder {k}: {e}"))?;
                            raw.push(Raw::Clamp {
                                p,
                                name: Support::holder_name(support, k),
                                moving,
                                reserve: 0.0,
                                attach: h.at.clone(),
                                kind: h.kind,
                                direction: world_direction(&pose, &h.direction),
                                rail: rail_world(&pose, h.kind, &h.axis, &h.stroke_m, &h.force_n),
                            });
                        }
                        carriers.push(support.clone());
                    }
                },
            }
        }
        // A duct is entered at the point of its centerline nearest the
        // previous way point and left at the point nearest the next — a
        // cable comes into a floor duct where it meets it, not at an end.
        let off = spec.od_m.unwrap_or(0.0) / 2.0
            + route
                .clamp
                .as_ref()
                .and_then(|c| c.standoff_m)
                .unwrap_or(DEFAULT_STANDOFF_M);
        let clamp_kind = route.clamp.as_ref().map(|c| c.kind).unwrap_or_default();
        let clamp_reserve = route
            .clamp
            .as_ref()
            .map(|c| c.length_m / 2.0)
            .unwrap_or(0.0);
        // Where the way continues to after `raw[i]`: a way point, a clamp,
        // the nearest point of the next duct, the next body's origin, or
        // the far stub.
        let next_of = |i: usize, prev: Point3<f64>, stub_b: Point3<f64>| -> Point3<f64> {
            match raw.get(i + 1) {
                Some(Raw::Point(p)) | Some(Raw::Clamp { p, .. }) => *p,
                Some(Raw::Duct { path: other, .. }) => closest_on_polyline(other, &prev).0,
                Some(Raw::Along { on, .. }) => self.body_origin(on).unwrap_or(stub_b),
                None => stub_b,
            }
        };
        type Expanded = (Vec<Vert>, Vec<String>, Vec<String>, Vec<String>);
        let expand = |stub_a: Point3<f64>, stub_b: Point3<f64>| -> Result<Expanded, String> {
            let mut middle: Vec<Vert> = Vec::new();
            let mut ducts = Vec::new();
            let mut clamps = Vec::new();
            let mut auto: Vec<String> = Vec::new();
            let mut prev = stub_a;
            for (i, item) in raw.iter().enumerate() {
                match item {
                    Raw::Point(p) => {
                        middle.push(Vert {
                            p: *p,
                            anchor: None,
                            fillet: true,
                            in_duct: false,
                            reserve: 0.0,
                        });
                        prev = *p;
                    }
                    Raw::Clamp {
                        p,
                        name: clamp,
                        moving,
                        reserve,
                        attach,
                        kind,
                        direction,
                        rail,
                    } => {
                        if Support::holder_of(clamp).is_none() {
                            clamps.push(clamp.clone());
                        }
                        middle.push(Vert {
                            p: *p,
                            anchor: Some(VertAnchor {
                                name: clamp.clone(),
                                kind: AnchorKind::Clamp,
                                moving: *moving,
                                exit: None,
                                attach: Some(attach.clone()),
                                auto: false,
                                holder: *kind,
                                direction: *direction,
                                rail: *rail,
                            }),
                            fillet: false,
                            in_duct: false,
                            reserve: *reserve,
                        });
                        prev = *p;
                    }
                    Raw::Duct { name: duct, path } => {
                        let next = next_of(i, prev, stub_b);
                        let (entry, s_in) = closest_on_polyline(path, &prev);
                        let (_, s_out) = closest_on_polyline(path, &next);
                        let run = polyline_between(path, s_in, s_out);
                        let count = run.len();
                        for (k, p) in run.into_iter().enumerate() {
                            middle.push(Vert {
                                p,
                                anchor: None,
                                fillet: true,
                                in_duct: k + 1 < count,
                                reserve: 0.0,
                            });
                        }
                        prev = middle.last().map(|v| v.p).unwrap_or(entry);
                        ducts.push(duct.clone());
                    }
                    Raw::Along {
                        on,
                        pitch,
                        face,
                        offset,
                        inset,
                    } => {
                        // The run's clamps, laid on the body; the line
                        // between two of them is drawn as it is.
                        let next = next_of(i, prev, stub_b);
                        let attaches = self
                            .dress_run(
                                on,
                                prev,
                                next,
                                off,
                                *pitch,
                                face.as_deref(),
                                *offset,
                                *inset,
                            )
                            .map_err(|e| format!("route `{name}`: {e}"))?;
                        let count = attaches.len();
                        for (j, at) in attaches.into_iter().enumerate() {
                            let (p, _, moving) = self
                                .attach_point(&at)
                                .map_err(|e| format!("route `{name}`: {e}"))?;
                            let clamp = format!("{name}/c{}", auto.len());
                            auto.push(clamp.clone());
                            middle.push(Vert {
                                p,
                                anchor: Some(VertAnchor {
                                    name: clamp,
                                    kind: AnchorKind::Clamp,
                                    moving,
                                    exit: None,
                                    attach: Some(at),
                                    auto: true,
                                    holder: clamp_kind,
                                    direction: None,
                                    rail: None,
                                }),
                                fillet: false,
                                in_duct: j + 1 < count,
                                reserve: clamp_reserve,
                            });
                            prev = p;
                        }
                    }
                }
            }
            Ok((middle, ducts, clamps, auto))
        };
        // The stubs: `straight_m` of line leaves each connector along its
        // exit, plus the tangent length the first corner's fillet will
        // take, so that exactly `straight_m` stays straight once the corner
        // is rounded. The corner's angle is read off the first move the
        // route makes from the stub (axis-aligned when orthogonal); the
        // duct entries move with the stubs, so it is settled in two passes.
        let tan_len = |theta: f64| -> f64 {
            match r_req {
                Some(r) if theta > 1e-6 && theta < PI - 1e-6 => r * (theta / 2.0).tan(),
                _ => 0.0,
            }
        };
        let angle = |a: Vector3<f64>, b: Vector3<f64>| a.dot(&b).clamp(-1.0, 1.0).acos();
        let tangents = |middle: &[Vert], stub_a: Point3<f64>, stub_b: Point3<f64>| -> (f64, f64) {
            let next_a = middle.first().map(|v| v.p).unwrap_or(stub_b);
            let prev_b = middle.last().map(|v| v.p).unwrap_or(stub_a);
            (
                first_move(stub_a, next_a)
                    .map(|d| tan_len(angle(ea, d)))
                    .unwrap_or(0.0),
                last_move(prev_b, stub_b)
                    .map(|d| tan_len(angle(d, -eb)))
                    .unwrap_or(0.0),
            )
        };
        let (mut stub_a, mut stub_b) = (pa + ea * geo_a.straight_m, pb + eb * geo_b.straight_m);
        let (middle, _, _, _) = expand(stub_a, stub_b)?;
        let (t_a, t_b) = tangents(&middle, stub_a, stub_b);
        stub_a = pa + ea * (geo_a.straight_m + t_a);
        stub_b = pb + eb * (geo_b.straight_m + t_b);
        let (middle, ducts, clamps, auto_clamps) = expand(stub_a, stub_b)?;
        let (t_a, t_b) = tangents(&middle, stub_a, stub_b);

        let mut verts: Vec<Vert> = Vec::with_capacity(middle.len() + 4);
        verts.push(Vert {
            p: pa,
            anchor: Some(VertAnchor {
                name: port_a.name.clone(),
                kind: AnchorKind::Port,
                moving: moving_a,
                exit: Some(ea),
                attach: Some(geo_a.at.clone()),
                auto: false,
                holder: HolderKind::Fixed,
                direction: None,
                rail: None,
            }),
            fillet: false,
            in_duct: false,
            reserve: geo_a.straight_m,
        });
        if geo_a.straight_m + t_a > EPS {
            verts.push(Vert {
                p: pa + ea * (geo_a.straight_m + t_a),
                anchor: None,
                fillet: true,
                in_duct: false,
                reserve: 0.0,
            });
        }
        verts.extend(middle);
        if geo_b.straight_m + t_b > EPS {
            verts.push(Vert {
                p: pb + eb * (geo_b.straight_m + t_b),
                anchor: None,
                fillet: true,
                in_duct: false,
                reserve: 0.0,
            });
        }
        verts.push(Vert {
            p: pb,
            anchor: Some(VertAnchor {
                name: port_b.name.clone(),
                kind: AnchorKind::Port,
                moving: moving_b,
                exit: Some(eb),
                attach: Some(geo_b.at.clone()),
                auto: false,
                holder: HolderKind::Fixed,
                direction: None,
                rail: None,
            }),
            fillet: false,
            in_duct: false,
            reserve: geo_b.straight_m,
        });

        if route.orthogonal {
            // Elbows between consecutive way points, z then x then y, for
            // the components that matter; what is below `tol` rides along
            // in the last move rather than becoming a jog of its own.
            let mut out: Vec<Vert> = Vec::with_capacity(verts.len() * 3);
            for i in 0..verts.len() {
                if i > 0 && !verts[i - 1].in_duct {
                    let a = out.last().map(|v| v.p).expect("previous vertex");
                    let b = verts[i].p;
                    let sig = significant(b - a);
                    if sig.len() >= 2 {
                        let mut cur = a;
                        for k in &sig[..sig.len() - 1] {
                            cur[*k] = b[*k];
                            out.push(Vert {
                                p: cur,
                                anchor: None,
                                fillet: true,
                                in_duct: false,
                                reserve: 0.0,
                            });
                        }
                    }
                }
                out.push(Vert {
                    p: verts[i].p,
                    anchor: verts[i].anchor.clone(),
                    fillet: verts[i].fillet,
                    in_duct: verts[i].in_duct,
                    reserve: verts[i].reserve,
                });
            }
            verts = out;
        }

        // Coincident neighbours collapse; an anchor survives the collapse.
        let mut dedup: Vec<Vert> = Vec::with_capacity(verts.len());
        for v in verts {
            match dedup.last_mut() {
                Some(last) if (last.p - v.p).norm() <= EPS => {
                    if last.anchor.is_none() {
                        last.anchor = v.anchor;
                    }
                    last.fillet = last.fillet && v.fillet;
                    last.in_duct = v.in_duct;
                    last.reserve = last.reserve.max(v.reserve);
                }
                _ => dedup.push(v),
            }
        }
        let verts = dedup;
        if verts.len() < 2 {
            return Err(format!(
                "route `{name}` has no length: both ends sit at one point"
            ));
        }

        let mut centerline: Vec<Point3<f64>> = Vec::new();
        let mut s: Vec<f64> = Vec::new();
        let mut length = 0.0;
        let mut anchors: Vec<RouteAnchor> = Vec::new();
        let mut bends: Vec<Bend> = Vec::new();
        let push = |p: Point3<f64>,
                    centerline: &mut Vec<Point3<f64>>,
                    s: &mut Vec<f64>,
                    length: &mut f64| {
            if let Some(last) = centerline.last() {
                let step = (p - last).norm();
                // Two fillets meeting exactly — a straight just long enough
                // for both tangents — would leave a zero-length segment,
                // which a replay's rope refuses.
                if step < 1e-9 {
                    return;
                }
                *length += step;
            }
            centerline.push(p);
            s.push(*length);
        };
        // The line's direction through a holder: a port's stub, an auto
        // clamp's run (the leg that is the run, at a run's first or last
        // clamp), else the bisector of the legs either side of it.
        let tangent_at = |i: usize| -> Option<[f64; 3]> {
            let leg = |a: usize, b: usize| {
                let d = verts[b].p - verts[a].p;
                (d.norm() > 1e-9).then(|| d.normalize())
            };
            let before = (i > 0).then(|| leg(i - 1, i)).flatten();
            let after = (i + 1 < verts.len()).then(|| leg(i, i + 1)).flatten();
            let dir = match (before, after) {
                (Some(u), Some(w)) => {
                    let on_run = verts[i].anchor.as_ref().is_some_and(|a| a.auto);
                    if on_run && verts[i].in_duct {
                        w
                    } else if on_run && verts[i - 1].in_duct {
                        u
                    } else {
                        let b = u + w;
                        if b.norm() > 1e-6 {
                            b.normalize()
                        } else {
                            u
                        }
                    }
                }
                (Some(u), None) => u,
                (None, Some(w)) => w,
                (None, None) => return None,
            };
            // A holder that declares its direction: that, the way the
            // line travels.
            let dir = match verts[i].anchor.as_ref().and_then(|a| a.direction) {
                Some(d) if d.dot(&dir) < 0.0 => -d,
                Some(d) => d,
                None => dir,
            };
            Some([dir.x, dir.y, dir.z])
        };
        let record_anchor = |i: usize, s_m: f64, anchors: &mut Vec<RouteAnchor>| {
            let v = &verts[i];
            if let Some(a) = &v.anchor {
                anchors.push(RouteAnchor {
                    name: a.name.clone(),
                    kind: a.kind,
                    position: to_array(v.p),
                    exit: a.exit.map(|e| [e.x, e.y, e.z]),
                    tangent: tangent_at(i),
                    moving: a.moving,
                    s_m,
                    attach: a.attach.clone(),
                    holder: a.holder,
                    rail: a.rail.map(|r| Rail {
                        axis: [r.axis.x, r.axis.y, r.axis.z],
                        stroke_m: r.stroke,
                        force_n: r.force,
                    }),
                    auto: a.auto,
                });
            }
        };
        push(verts[0].p, &mut centerline, &mut s, &mut length);
        record_anchor(0, 0.0, &mut anchors);
        let n = verts.len();
        for i in 1..n - 1 {
            let u = verts[i].p - verts[i - 1].p;
            let w = verts[i + 1].p - verts[i].p;
            let (lu, lw) = (u.norm(), w.norm());
            if (lu < MICRO_JOG_M || lw < MICRO_JOG_M) && verts[i].anchor.is_none() {
                // A jog of a millimetre or two: no fillet fits on it and
                // the line does not care — drawn through, not judged.
                push(verts[i].p, &mut centerline, &mut s, &mut length);
                continue;
            }
            let cos = (u.dot(&w) / (lu * lw)).clamp(-1.0, 1.0);
            let theta = cos.acos();
            if theta < 1e-6 {
                push(verts[i].p, &mut centerline, &mut s, &mut length);
                record_anchor(i, length, &mut anchors);
                continue;
            }
            let corner_s = length + lu;
            let hairpin = theta > PI - 1e-6;
            // How much of each neighbouring run a fillet here may take: half
            // (the other half is the next corner's) — or, next to an anchor,
            // everything but what the anchor keeps straight.
            let avail = |len: f64, neighbour: &Vert| -> f64 {
                if neighbour.fillet {
                    len / 2.0
                } else {
                    (len - neighbour.reserve).max(0.0)
                }
            };
            let t_avail = avail(lu, &verts[i - 1]).min(avail(lw, &verts[i + 1]));
            if !verts[i].fillet || r_req.is_none() || hairpin || t_avail <= EPS {
                push(verts[i].p, &mut centerline, &mut s, &mut length);
                record_anchor(i, length, &mut anchors);
                if verts[i].anchor.is_some() {
                    // A turn at a clamp: the holder's orientation is not
                    // modelled, so the line is drawn through the point and
                    // the corner is nobody's finding.
                    continue;
                }
                // A free corner with no radius to draw is a sharp bend the
                // checks cannot judge; a hairpin or a corner with no room
                // for its fillet is a bend that fails.
                let fails = hairpin || t_avail <= EPS;
                bends.push(Bend {
                    position: to_array(verts[i].p),
                    s_m: corner_s,
                    angle_deg: theta.to_degrees(),
                    radius_m: if fails { Some(0.0) } else { None },
                    required_m: r_req,
                    ok: if fails { r_req.map(|_| false) } else { None },
                });
                continue;
            }
            let r = r_req.expect("checked above");
            let tan_half = (theta / 2.0).tan();
            let r_max = t_avail / tan_half;
            // Within 2 % the fillet is the required one: the stubs were
            // sized from a first guess of the corner's angle, and a hair of
            // difference is not a bend that fails.
            let ok = r <= r_max * 1.02 + 1e-9;
            let r_used = r.min(r_max);
            let t = r_used * tan_half;
            let uh = u / lu;
            let wh = w / lw;
            let t1 = verts[i].p - uh * t;
            let t2 = verts[i].p + wh * t;
            let bis = (wh - uh).normalize();
            let c = verts[i].p + bis * (r_used / (theta / 2.0).cos());
            let axis = Unit::new_normalize(uh.cross(&wh));
            let segs = ((theta / ARC_STEP).ceil() as usize).max(2);
            push(t1, &mut centerline, &mut s, &mut length);
            let arc_start = length;
            for k in 1..segs {
                let rot = Rotation3::from_axis_angle(&axis, theta * k as f64 / segs as f64);
                push(c + rot * (t1 - c), &mut centerline, &mut s, &mut length);
            }
            push(t2, &mut centerline, &mut s, &mut length);
            bends.push(Bend {
                position: to_array(verts[i].p),
                s_m: (arc_start + length) / 2.0,
                angle_deg: theta.to_degrees(),
                radius_m: Some(r_used),
                required_m: Some(r),
                ok: Some(ok),
            });
        }
        push(verts[n - 1].p, &mut centerline, &mut s, &mut length);
        record_anchor(n - 1, length, &mut anchors);
        // The run inside the ducts: the segments laid along their centerlines.
        let ducted_m: f64 = verts
            .windows(2)
            .filter(|w| w[0].in_duct)
            .map(|w| (w[1].p - w[0].p).norm())
            .sum();

        let spans = anchors
            .windows(2)
            .map(|pair| Span {
                kind: if !(pair[0].moving || pair[1].moving) {
                    SpanKind::Fixed
                } else if self.same_body(pair[0].attach.as_ref(), pair[1].attach.as_ref()) {
                    SpanKind::Dressed
                } else {
                    SpanKind::Moving
                },
                from: pair[0].name.clone(),
                to: pair[1].name.clone(),
                from_s: pair[0].s_m,
                to_s: pair[1].s_m,
                slides: pair[0].holder.slides(),
            })
            .collect();

        let cut = length + 2.0 * spec.end_allowance_m + route.slack_total_m();
        let purchase = if !spec.sold_lengths_m.is_empty() {
            let mut sold = spec.sold_lengths_m.clone();
            sold.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
            sold.into_iter().find(|v| *v + 1e-9 >= cut)
        } else {
            spec.order_step_m
                .map(|step| (cut / step - 1e-9).ceil().max(1.0) * step)
        };
        Ok(RouteGeometry {
            name: route.name.clone(),
            spec: spec.name.clone(),
            kind: spec.kind,
            medium,
            carries: route.carries.clone(),
            from: endpoint(port_a),
            to: endpoint(port_b),
            centerline: centerline.into_iter().map(to_array).collect(),
            s,
            anchors,
            spans,
            bends,
            ducts,
            clamps,
            carriers,
            auto_clamps,
            length_m: length,
            ducted_m,
            cut_length_m: cut,
            purchase_length_m: purchase,
            od_m: spec.od_m,
            color: route
                .color
                .or(spec.color)
                .unwrap_or_else(|| medium_color(medium)),
        })
    }

    /// The cable list and the static checks over every route.
    pub fn routing_report(&self) -> RoutingReport {
        let plan = &self.routing_plan;
        let mut findings: Vec<RoutingFinding> = Vec::new();
        let mut geometries: Vec<Option<RouteGeometry>> = Vec::new();
        let mut rows: Vec<RouteRow> = Vec::new();
        let finding = |severity: &str,
                       code: &str,
                       route: &str,
                       message: String,
                       position: Option<[f64; 3]>,
                       s_m: Option<f64>,
                       findings: &mut Vec<RoutingFinding>| {
            findings.push(RoutingFinding {
                severity: severity.to_string(),
                code: code.to_string(),
                route: route.to_string(),
                message,
                position,
                s_m,
                t_s: None,
                rope: None,
                sample: None,
            });
        };

        for route in &plan.routes {
            let spec = plan.spec(&route.spec);
            let before = findings.len();
            let geometry = match self.route_geometry(&route.name) {
                Ok(g) => Some(g),
                Err(e) => {
                    finding(
                        "error",
                        "route_endpoint",
                        &route.name,
                        e,
                        None,
                        None,
                        &mut findings,
                    );
                    None
                }
            };
            if let Some(spec) = spec {
                // Unknown values read as unknown; nothing is assumed.
                let mut unknown = Vec::new();
                if spec.od_m.is_none() {
                    unknown.push("od_m");
                }
                if spec.bend_r_fixed_m.is_none() {
                    unknown.push("bend_r_fixed_m");
                }
                if spec.bend_r_moving_m.is_none()
                    && geometry
                        .as_ref()
                        .is_some_and(|g| g.spans.iter().any(|s| s.kind == SpanKind::Moving))
                {
                    unknown.push("bend_r_moving_m");
                }
                if !unknown.is_empty() {
                    finding(
                        "warning",
                        "route_spec_unknown",
                        &route.name,
                        format!(
                            "spec `{}` states no {} — the checks that need them stay unknown",
                            spec.name,
                            unknown.join(", ")
                        ),
                        None,
                        None,
                        &mut findings,
                    );
                }
                // Every carried connection must be a medium the line carries.
                for carried in &route.carries {
                    let Ok(connection) = self.connection(carried) else {
                        finding(
                            "error",
                            "route_endpoint",
                            &route.name,
                            format!("carries an unknown connection `{carried}`"),
                            None,
                            None,
                            &mut findings,
                        );
                        continue;
                    };
                    let medium = self
                        .connection_plan()
                        .ports
                        .iter()
                        .find(|p| p.name == connection.source)
                        .map(|p| p.medium);
                    if let Some(medium) = medium {
                        if !spec.media.is_empty() && !spec.media.contains(&medium) {
                            finding(
                                "error",
                                "route_medium",
                                &route.name,
                                format!(
                                    "carries `{carried}` ({}) on spec `{}`, which carries only {}",
                                    medium_name(medium),
                                    spec.name,
                                    spec.media
                                        .iter()
                                        .map(|m| medium_name(*m))
                                        .collect::<Vec<_>>()
                                        .join(", ")
                                ),
                                None,
                                None,
                                &mut findings,
                            );
                        }
                    }
                }
            }
            if let Some(g) = &geometry {
                self.check_geometry(route, spec, g, &mut |sev, code, msg, pos, s_m| {
                    finding(sev, code, &route.name, msg, pos, s_m, &mut findings)
                });
            }
            let mine = &findings[before..];
            let status = if mine.iter().any(|f| f.severity == "error") {
                "error"
            } else if mine.iter().any(|f| f.severity == "warning") {
                "warning"
            } else {
                "ok"
            };
            rows.push(RouteRow {
                name: route.name.clone(),
                spec: route.spec.clone(),
                kind: spec.map(|s| s.kind).unwrap_or_default(),
                medium: geometry.as_ref().and_then(|g| g.medium),
                carries: route.carries.clone(),
                from: geometry.as_ref().map(|g| g.from.clone()),
                to: geometry.as_ref().map(|g| g.to.clone()),
                manufacturer: spec.and_then(|s| s.manufacturer.clone()),
                model: spec.and_then(|s| s.model.clone()),
                catalog: spec.and_then(|s| s.catalog.clone()),
                od_mm: spec.and_then(|s| s.od_m).map(|v| (v * 1e4).round() / 10.0),
                length_m: geometry.as_ref().map(|g| round_m(g.length_m)),
                fixed_m: geometry.as_ref().map(|g| round_m(g.fixed_length_m())),
                moving_m: geometry.as_ref().map(|g| round_m(g.moving_length_m())),
                cut_length_m: geometry.as_ref().map(|g| round_m(g.cut_length_m)),
                purchase_length_m: geometry
                    .as_ref()
                    .and_then(|g| g.purchase_length_m)
                    .map(round_m),
                ducts: geometry
                    .as_ref()
                    .map(|g| g.ducts.clone())
                    .unwrap_or_default(),
                clamps: geometry
                    .as_ref()
                    .map(|g| g.clamps.clone())
                    .unwrap_or_default(),
                carriers: geometry
                    .as_ref()
                    .map(|g| g.carriers.clone())
                    .unwrap_or_default(),
                auto_clamps: geometry.as_ref().map(|g| g.auto_clamps.len()).unwrap_or(0),
                bends: geometry.as_ref().map(|g| g.bends.len()).unwrap_or(0),
                status: status.to_string(),
                findings: mine.len(),
            });
            geometries.push(geometry);
        }

        // Bundles: what shares a duct or a carrier must fit its section, and
        // power does not run with signal or network lines in one.
        for support in &plan.supports {
            let (inner_area, what) = match support {
                Support::Duct { inner_m, .. } => (inner_m[0] * inner_m[1], "duct"),
                Support::Carrier {
                    inner_m: Some(d), ..
                } => (PI * d * d / 4.0, "carrier"),
                // A bore nobody states: the requirement row says so.
                Support::Carrier { inner_m: None, .. } | Support::Clamp { .. } => continue,
            };
            let name = support.name();
            let through: Vec<&RouteGeometry> = geometries
                .iter()
                .flatten()
                .filter(|g| {
                    g.ducts.iter().any(|d| d == name) || g.carriers.iter().any(|c| c == name)
                })
                .collect();
            if through.is_empty() {
                continue;
            }
            let mut area = 0.0;
            let mut unknown: Vec<String> = Vec::new();
            let mut media: Vec<Medium> = Vec::new();
            for g in &through {
                match g.od_m {
                    Some(od) => area += PI * od * od / 4.0,
                    None => unknown.push(g.name.clone()),
                }
                if let Some(m) = g.medium {
                    if !media.contains(&m) {
                        media.push(m);
                    }
                }
            }
            let fill = area / inner_area;
            if fill > FILL_LIMIT {
                // Past the limit a bundle is hard to pull; past the whole
                // section it does not go in at all.
                finding(
                    if fill > 1.0 { "error" } else { "warning" },
                    "duct_fill",
                    name,
                    format!(
                        "`{name}` ({what}) is {:.0} % full by cross-section with {} line(s) (limit {:.0} %){}",
                        fill * 100.0,
                        through.len(),
                        FILL_LIMIT * 100.0,
                        if unknown.is_empty() {
                            String::new()
                        } else {
                            format!(" — {} of unknown diameter not counted", unknown.join(", "))
                        }
                    ),
                    None,
                    None,
                    &mut findings,
                );
            }
            if media.contains(&Medium::Power)
                && (media.contains(&Medium::Signal) || media.contains(&Medium::Network))
            {
                finding(
                    "warning",
                    "route_separation",
                    name,
                    format!("`{name}` ({what}) carries power together with signal / network lines — separate them"),
                    None,
                    None,
                    &mut findings,
                );
            }
        }

        // Fixed spans of different routes crossing each other, unless both
        // run in a shared duct (a bundle is not a crossing).
        let resolved: Vec<&RouteGeometry> = geometries.iter().flatten().collect();
        for (i, a) in resolved.iter().enumerate() {
            for b in resolved.iter().skip(i + 1) {
                if a.ducts.iter().any(|d| b.ducts.contains(d)) {
                    continue;
                }
                if let Some(point) = fixed_spans_touch(a, b) {
                    finding(
                        "warning",
                        "route_crossing",
                        &a.name,
                        format!("crosses route `{}`", b.name),
                        Some(point),
                        None,
                        &mut findings,
                    );
                }
            }
        }

        // Connections with both ends placed and no route yet: the to-do.
        for connection in &self.connection_plan().links {
            let placed = |name: &str| {
                self.connection_plan()
                    .ports
                    .iter()
                    .any(|p| p.name == name && p.geometry.is_some())
            };
            if placed(&connection.source)
                && placed(&connection.target)
                && !plan
                    .routes
                    .iter()
                    .any(|r| r.carries.contains(&connection.name))
            {
                finding(
                    "info",
                    "route_unrouted",
                    &connection.name,
                    format!(
                        "connection `{}` has both ports placed but no route carries it",
                        connection.name
                    ),
                    None,
                    None,
                    &mut findings,
                );
            }
        }
        RoutingReport {
            routes: rows,
            findings,
            geometries,
        }
    }

    /// Bends, exits, lengths and clashes of one resolved route.
    fn check_geometry(
        &self,
        route: &Route,
        spec: Option<&CableSpec>,
        g: &RouteGeometry,
        finding: &mut FindingSink<'_>,
    ) {
        for bend in &g.bends {
            if bend.ok == Some(false) {
                let required = bend.required_m.unwrap_or(0.0);
                let message = match bend.radius_m {
                    Some(r) if r > 0.0 => format!(
                        "a {:.0}° bend at {:.2} m gets r = {:.0} mm; the line needs r ≥ {:.0} mm — the neighbouring runs are too short for the fillet",
                        bend.angle_deg,
                        bend.s_m,
                        r * 1000.0,
                        required * 1000.0
                    ),
                    _ => format!(
                        "a sharp {:.0}° bend at {:.2} m; the line needs r ≥ {:.0} mm",
                        bend.angle_deg,
                        bend.s_m,
                        required * 1000.0
                    ),
                };
                finding(
                    "error",
                    "route_bend",
                    message,
                    Some(bend.position),
                    Some(bend.s_m),
                );
            }
        }
        // The line leaves each connector along its exit direction.
        let n = g.centerline.len();
        if n >= 2 {
            let checks = [
                (&g.anchors[0], g.centerline[0], g.centerline[1]),
                (
                    &g.anchors[g.anchors.len() - 1],
                    g.centerline[n - 1],
                    g.centerline[n - 2],
                ),
            ];
            for (anchor, at, next) in checks {
                let Some(exit) = anchor.exit else { continue };
                let dir = Vector3::from(next) - Vector3::from(at);
                let Some(dir) = dir.try_normalize(EPS) else {
                    continue;
                };
                let angle = dir
                    .dot(&Vector3::from(exit))
                    .clamp(-1.0, 1.0)
                    .acos()
                    .to_degrees();
                if angle > EXIT_TOLERANCE_DEG {
                    finding(
                        "warning",
                        "route_exit",
                        format!(
                            "leaves port `{}` {angle:.0}° off its exit direction",
                            anchor.name
                        ),
                        Some(anchor.position),
                        Some(anchor.s_m),
                    );
                }
            }
        }
        if let Some(spec) = spec {
            if let Some(max) = spec.max_length_m {
                if g.cut_length_m > max + 1e-9 {
                    finding(
                        "error",
                        "route_length",
                        format!(
                            "cut length {:.2} m exceeds the spec's maximum run of {:.2} m",
                            g.cut_length_m, max
                        ),
                        None,
                        None,
                    );
                }
            }
            if !spec.sold_lengths_m.is_empty() && g.purchase_length_m.is_none() {
                finding(
                    "error",
                    "route_length",
                    format!(
                        "cut length {:.2} m fits none of the sold lengths ({})",
                        g.cut_length_m,
                        spec.sold_lengths_m
                            .iter()
                            .map(|v| format!("{v} m"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                    None,
                    None,
                );
            }
        }
        if g.spans.iter().any(|s| s.kind == SpanKind::Moving) && route.slack_total_m() <= 0.0 {
            finding(
                "warning",
                "route_length",
                "has a moving span and no slack (`slack_m`): a span that moves needs a service loop".to_string(),
                None,
                None,
            );
        }
        self.check_design_rules(route, spec, g, finding);
        self.check_clashes(route, g, finding);
    }

    /// The rules of a dress pack's fixing, on the plan as laid: a holder
    /// right after a connector on another body (the connector takes the
    /// bend), a holder on a joint housing, a cable's moving span held by
    /// its connector alone, and a joint that turns the line about its own
    /// axis between two holders with less twist allowed than its full
    /// swing wants.
    fn check_design_rules(
        &self,
        route: &Route,
        spec: Option<&CableSpec>,
        g: &RouteGeometry,
        finding: &mut FindingSink<'_>,
    ) {
        let ports = &self.connection_plan().ports;
        let radius = g.od_m.map(|d| d / 2.0).unwrap_or(0.0);
        let bend_fixed = spec.and_then(|s| s.bend_r_fixed_m).unwrap_or(0.0);
        let anchor = |name: &str| g.anchors.iter().find(|a| a.name == name);
        // The first holder after a connector, on another body, within the
        // stub's reach: the line bends right behind the connector.
        for (i, a) in g.anchors.iter().enumerate() {
            if a.kind != AnchorKind::Port {
                continue;
            }
            let Some(port) = ports.iter().find(|p| p.name == a.name) else {
                continue;
            };
            let straight = port.geometry.as_ref().map(|q| q.straight_m).unwrap_or(0.0);
            let next = if i == 0 {
                g.anchors.get(1)
            } else if i + 1 == g.anchors.len() {
                g.anchors.get(i - 1)
            } else {
                None
            };
            let Some(next) = next else { continue };
            if next.kind == AnchorKind::Port
                || self.same_body(a.attach.as_ref(), next.attach.as_ref())
            {
                continue;
            }
            let apart = (next.s_m - a.s_m).abs();
            let reach = straight + bend_fixed + 2.0 * radius;
            if apart + 1e-9 < reach {
                finding(
                    "warning",
                    "route_exit_bend",
                    format!(
                        "the first holder after {} ({}) is {:.0} mm along the line, within its {:.0} mm stub and a bend radius: the connector takes the bend — move the holder out or lengthen the stub",
                        a.name, next.name, apart * 1000.0, reach * 1000.0
                    ),
                    Some(next.position),
                    Some(next.s_m),
                );
            }
        }
        // A declared holder on a joint housing: the joint turns under it.
        for a in &g.anchors {
            if a.kind != AnchorKind::Clamp || a.auto {
                continue;
            }
            let Some(Attach::Link { robot, link, .. }) = &a.attach else {
                continue;
            };
            let Some(r) = self.robot_index(robot) else {
                continue;
            };
            let model = &self.robots()[r].model;
            let Some(l) = model.link_index(link) else {
                continue;
            };
            let poses = self.link_poses_for(r);
            let p = Point3::from(a.position);
            for joint in &model.joints {
                if joint.joint_type == botrail_model::JointType::Fixed
                    || (joint.parent_link != l && joint.child_link != l)
                {
                    continue;
                }
                let frame = &poses[joint.child_link];
                let axis = frame.rotation * joint.axis.into_inner();
                let d = p - Point3::from(frame.translation.vector);
                let along = d.dot(&axis);
                let radial = (d - axis * along).norm();
                if radial < JOINT_ZONE_M && along.abs() < JOINT_ZONE_M {
                    finding(
                        "warning",
                        "route_clamp_near_joint",
                        format!(
                            "holder {} sits on the housing of joint {} ({:.0} mm from its axis): the joint turns under it — move it along the link",
                            a.name, joint.name, radial * 1000.0
                        ),
                        Some(a.position),
                        Some(a.s_m),
                    );
                }
            }
        }
        // A cable's moving span ending at its connector on the moving body:
        // the connector holds it, nothing relieves the strain (a tube's
        // push-in fitting is its holder).
        for span in g.spans.iter().filter(|s| s.kind == SpanKind::Moving) {
            for name in [&span.from, &span.to] {
                let Some(a) = anchor(name) else { continue };
                if a.kind != AnchorKind::Port || !a.moving {
                    continue;
                }
                let Some(port) = ports.iter().find(|p| p.name == a.name) else {
                    continue;
                };
                if port.medium == Medium::Pneumatic {
                    continue;
                }
                finding(
                    "warning",
                    "route_connector_holds",
                    format!(
                        "the moving span {} → {} ends at the connector {}: nothing relieves its strain — a clamp on the same body before it",
                        span.from, span.to, a.name
                    ),
                    Some(a.position),
                    Some(a.s_m),
                );
            }
        }
        // A joint between a moving span's holders that turns the line about
        // its own axis: room for ±180° at each (a cobot's tool line is dressed
        // for it) against the twist the line allows over the span.
        let Some(torsion) = spec.and_then(|s| s.torsion_deg_m) else {
            return;
        };
        let moving = g
            .spans
            .iter()
            .filter(|s| s.kind == SpanKind::Moving)
            .count();
        for (seen, span) in g
            .spans
            .iter()
            .filter(|s| s.kind == SpanKind::Moving)
            .enumerate()
        {
            let share = route.slack_of_moving(seen, moving);
            let material = (span.to_s - span.from_s) + share;
            let (Some(a), Some(b)) = (anchor(&span.from), anchor(&span.to)) else {
                continue;
            };
            let link_of = |x: &RouteAnchor| match &x.attach {
                Some(Attach::Link { robot, link, .. }) => self
                    .robot_index(robot)
                    .and_then(|r| self.robots()[r].model.link_index(link).map(|l| (r, l))),
                _ => None,
            };
            let (la, lb) = (link_of(a), link_of(b));
            let r = match (la, lb) {
                (Some((ra, _)), Some((rb, _))) if ra == rb => ra,
                (Some((r, _)), None) | (None, Some((r, _))) => r,
                _ => continue,
            };
            let model = &self.robots()[r].model;
            let path = |mut l: usize| -> Vec<usize> {
                let mut joints = Vec::new();
                while let Some((j, joint)) = model
                    .joints
                    .iter()
                    .enumerate()
                    .find(|(_, j)| j.child_link == l)
                {
                    joints.push(j);
                    l = joint.parent_link;
                }
                joints
            };
            let pa = la.map(|(_, l)| path(l)).unwrap_or_default();
            let pb = lb.map(|(_, l)| path(l)).unwrap_or_default();
            let between: Vec<usize> = pa
                .iter()
                .filter(|j| !pb.contains(j))
                .chain(pb.iter().filter(|j| !pa.contains(j)))
                .copied()
                .collect();
            // The line's directions at the holders (its chord when a holder
            // has none): a joint whose axis runs with one of them turns the
            // line about its own axis; one square to both bends it.
            let chord = Vector3::from(b.position) - Vector3::from(a.position);
            if chord.norm() < 1e-9 {
                continue;
            }
            let chord = chord.normalize();
            let runs: Vec<Vector3<f64>> = [a, b]
                .iter()
                .map(|x| x.tangent.map(Vector3::from).unwrap_or(chord))
                .collect();
            let poses = self.link_poses_for(r);
            let twisters: Vec<&str> = between
                .iter()
                .map(|&j| &model.joints[j])
                .filter(|joint| {
                    matches!(
                        joint.joint_type,
                        botrail_model::JointType::Revolute | botrail_model::JointType::Continuous
                    ) && joint.q_index.is_some()
                })
                .filter(|joint| {
                    let axis = poses[joint.child_link].rotation * joint.axis.into_inner();
                    runs.iter().any(|run| axis.dot(run).abs() > 0.7)
                })
                .map(|joint| joint.name.as_str())
                .collect();
            if twisters.is_empty() {
                continue;
            }
            let room = TWIST_ROOM_DEG * twisters.len() as f64;
            let allowed = torsion * material;
            if allowed + 1e-9 < room {
                finding(
                    "warning",
                    "route_twist_room",
                    format!(
                        "between {} and {} the joint{} {} turn{} the line about its own axis: room for ±{:.0}° at each wants {:.0}° of twist, the line allows {:.0}°/m over {:.2} m = {:.0}° — a swivel, a rotating holder or more line",
                        span.from,
                        span.to,
                        if twisters.len() == 1 { "" } else { "s" },
                        twisters.join(", "),
                        if twisters.len() == 1 { "s" } else { "" },
                        TWIST_ROOM_DEG,
                        room,
                        torsion,
                        material,
                        allowed
                    ),
                    Some(a.position),
                    Some(a.s_m),
                );
            }
        }
    }

    /// Fixed spans against the enabled obstacles (less the ducts the route
    /// runs in and the residents its anchors sit on) and against every
    /// robot at its current configuration (less the links its anchors sit
    /// on).
    fn check_clashes(&self, route: &Route, g: &RouteGeometry, finding: &mut FindingSink<'_>) {
        let radius = g.od_m.map(|d| d / 2.0).unwrap_or(0.0);
        // Obstacles the route is meant to touch: the residents its anchors
        // sit on. Robot links are judged span by span, by how its ends are
        // held (`HeldEnd`).
        let mut skip_obstacles: BTreeSet<String> = BTreeSet::new();
        for anchor in &g.anchors {
            if let Some(Attach::Obstacle { name, .. }) = self.anchor_attach(anchor) {
                skip_obstacles.insert(name);
            }
        }
        let duct_prefixes: Vec<String> = g.ducts.iter().map(|d| format!("{d}/")).collect();
        let skip_obstacle = |name: &str| {
            skip_obstacles.contains(name)
                || duct_prefixes.iter().any(|p| name.starts_with(p))
                || g.ducts.iter().any(|d| d == name)
        };

        // A clamp or a holder on a robot link that holds the line inside the
        // link's shape: a clip sits on the surface, and a line held inside
        // is one the replay and the checks cannot judge against that link.
        for anchor in g.anchors.iter().filter(|a| a.kind == AnchorKind::Clamp) {
            if !self.held_end(anchor, Some(route), radius).holds {
                continue;
            }
            if let Some(Attach::Link { robot, link, .. }) = self.anchor_attach(anchor) {
                finding(
                    "warning",
                    "route_clash",
                    format!(
                        "clamp `{}` holds the line inside the robot link `{robot}/{link}` — a clip sits on the surface; held inside, the line is neither collided with that link in a replay nor judged against it",
                        anchor.name
                    ),
                    Some(anchor.position),
                    Some(anchor.s_m),
                );
            }
        }
        // Every span at the scene's pose — a span that rides a robot too:
        // a design line through the arm is caught before any replay.
        for span in &g.spans {
            let points = g.span_points(span);
            if points.len() < 2 {
                continue;
            }
            let (ends, dressed) = self.span_ends(g, span, Some(route), radius);
            let Some(nearest) = self.nearest_body_held(
                &points,
                radius,
                &skip_obstacle,
                &ends,
                dressed.as_ref(),
                None,
            ) else {
                continue;
            };
            // Where along the route: the span's start plus the way to the
            // sample nearest the place (the studio tells a finding on a
            // span that rides the arm by it).
            let s_m = span.from_s
                + points[..=nearest_sample(&points, nearest.point)]
                    .windows(2)
                    .map(|w| (Vector3::from(w[1]) - Vector3::from(w[0])).norm())
                    .sum::<f64>();
            if nearest.distance <= 1e-6 {
                finding(
                    "error",
                    "route_clash",
                    format!(
                        "span {} → {} runs into {}",
                        span.from, span.to, nearest.body
                    ),
                    Some(nearest.point),
                    Some(s_m),
                );
            } else if nearest.distance < CLEARANCE_MARGIN_M {
                finding(
                    "warning",
                    "route_clearance",
                    format!(
                        "span {} → {} passes {:.0} mm from {}",
                        span.from,
                        span.to,
                        nearest.distance * 1000.0,
                        nearest.body
                    ),
                    Some(nearest.point),
                    Some(s_m),
                );
            }
        }
    }

    /// The distance from a point to a robot link's collision shape (0 on or
    /// inside it — the parts are convex) at the given link poses; `None`
    /// for a link without one.
    fn link_point_distance(
        &self,
        r: usize,
        l: usize,
        poses: &[Isometry3<f64>],
        point: &Point3<f64>,
    ) -> Option<f64> {
        let parts = self.robots()[r].collider.link_parts(l);
        if parts.is_empty() {
            return None;
        }
        let probe = parry3d_f64::shape::Ball::new(1e-9);
        let at = botrail_collide::to_parry_pose(&Isometry3::translation(point.x, point.y, point.z));
        let world = botrail_collide::to_parry_pose(&poses[l]);
        parts
            .iter()
            .filter_map(|(part_pose, shape)| {
                parry3d_f64::query::distance(&at, &probe, &(world * *part_pose), shape.as_ref())
                    .ok()
            })
            .fold(None, |acc: Option<f64>, d| {
                Some(acc.map_or(d, |a| a.min(d)))
            })
    }

    /// The line an anchor holds straight: a port's stub, half a clamp (an
    /// auto clamp: half the route's clamp article), nothing at a holder.
    fn held_length(&self, anchor: &RouteAnchor, route: Option<&Route>) -> f64 {
        match anchor.kind {
            AnchorKind::Port => self
                .connection_plan()
                .ports
                .iter()
                .find(|p| p.name == anchor.name)
                .and_then(|p| p.geometry.as_ref())
                .map(|g| g.straight_m)
                .unwrap_or(0.0),
            AnchorKind::Clamp if anchor.auto => route
                .and_then(|r| r.clamp.as_ref())
                .map(|c| c.length_m / 2.0)
                .unwrap_or(0.0),
            AnchorKind::Clamp => match self.routing_plan.support(&anchor.name) {
                Some(Support::Clamp { length_m, .. }) => length_m / 2.0,
                _ => 0.0,
            },
        }
    }

    /// How the body under one end of a span takes part in its checks.
    fn held_end(&self, anchor: &RouteAnchor, route: Option<&Route>, radius: f64) -> HeldEnd {
        let Some(Attach::Link {
            robot,
            link,
            offset_m,
        }) = self.anchor_attach(anchor)
        else {
            return HeldEnd::default();
        };
        let Some(r) = self.robot_index(&robot) else {
            return HeldEnd::default();
        };
        let model = &self.robots()[r].model;
        let Some(l) = model.link_index(&link) else {
            return HeldEnd::default();
        };
        let poses = self.link_poses_for(r);
        let point = poses[l] * Point3::from(offset_m);
        let mut group: Vec<(usize, usize)> =
            rigid_group(model, l).into_iter().map(|m| (r, m)).collect();
        group.sort_unstable();
        let holds = group.iter().any(|&(r, m)| {
            self.link_point_distance(r, m, &poses, &point)
                .is_some_and(|d| d <= radius + 1e-6)
        });
        HeldEnd {
            group,
            holds,
            near_m: self.held_length(anchor, route) + 2.0 * radius + HELD_NEAR_M,
        }
    }

    /// The two ends of a span as its checks see them, and the link group a
    /// dressed span is laid along (both ends on one body: the surface it
    /// lies on is not a body it clashes with).
    fn span_ends(
        &self,
        g: &RouteGeometry,
        span: &Span,
        route: Option<&Route>,
        radius: f64,
    ) -> ([HeldEnd; 2], Option<Vec<(usize, usize)>>) {
        let end = |name: &str| {
            g.anchors
                .iter()
                .find(|a| a.name == name)
                .map(|a| self.held_end(a, route, radius))
                .unwrap_or_default()
        };
        let ends = [end(&span.from), end(&span.to)];
        let dressed = (span.kind == SpanKind::Dressed
            && !ends[0].group.is_empty()
            && ends[0].group == ends[1].group)
            .then(|| ends[0].group.clone());
        (ends, dressed)
    }

    /// The nearest body to a run of line held at its two ends. A holding
    /// link group whose shape holds the held point is the line's mount and
    /// is not looked at, nor is the group a dressed span is laid along; a
    /// holding group that does not hold it is judged except within `near_m`
    /// of its end (the clip, the connector's stub); every other link and
    /// obstacle as [`Scene::nearest_body`] judges them.
    fn nearest_body_held(
        &self,
        points: &[[f64; 3]],
        radius: f64,
        skip_obstacle: &dyn Fn(&str) -> bool,
        ends: &[HeldEnd; 2],
        dressed: Option<&Vec<(usize, usize)>>,
        near: Option<([f64; 3], [f64; 3])>,
    ) -> Option<Nearest> {
        let mut s = vec![0.0];
        for w in points.windows(2) {
            let step = (Vector3::from(w[1]) - Vector3::from(w[0])).norm();
            s.push(s.last().copied().unwrap_or(0.0) + step);
        }
        let total = s.last().copied().unwrap_or(0.0);
        let segments = |from: f64, to: f64| -> Vec<(Point3<f64>, Point3<f64>)> {
            points
                .windows(2)
                .zip(s.windows(2))
                .filter(|(_, sw)| sw[0] >= from - 1e-9 && sw[1] <= to + 1e-9)
                .map(|(w, _)| (Point3::from(w[0]), Point3::from(w[1])))
                .filter(|(a, b)| (b - a).norm() > EPS)
                .collect()
        };
        let mut skip: BTreeSet<(usize, usize)> = BTreeSet::new();
        for e in ends {
            skip.extend(e.group.iter().copied());
        }
        if let Some(d) = dressed {
            skip.extend(d.iter().copied());
        }
        let mut best = self.nearest_body(&segments(0.0, total), radius, skip_obstacle, &skip, near);
        for (k, e) in ends.iter().enumerate() {
            if e.group.is_empty() || e.holds || dressed.is_some_and(|d| *d == e.group) {
                continue;
            }
            let (from, to) = if k == 0 {
                (e.near_m, total)
            } else {
                (0.0, total - e.near_m)
            };
            if to <= from {
                continue;
            }
            let others: BTreeSet<(usize, usize)> = (0..self.robots().len())
                .flat_map(|r| (0..self.robots()[r].model.links.len()).map(move |l| (r, l)))
                .filter(|rl| !e.group.contains(rl))
                .collect();
            if let Some(n) =
                self.nearest_body(&segments(from, to), radius, &|_| true, &others, near)
            {
                if best.as_ref().is_none_or(|b| n.distance < b.distance) {
                    best = Some(Nearest { own: true, ..n });
                }
            }
        }
        best
    }

    /// The robot links a rope replaying span `k` of `route` collides with —
    /// every link with collision geometry of each robot one of its ends is
    /// held on — and the holding link groups left out because their shape
    /// holds the held point (a clip seated in the casting), by name.
    pub fn span_replay_links(
        &self,
        route: &str,
        k: usize,
        radius: f64,
    ) -> Result<ReplayLinks, String> {
        let g = self.route_geometry(route)?;
        let span = g
            .spans
            .get(k)
            .ok_or_else(|| format!("route `{route}` has no span {k}"))?;
        let (ends, _) = self.span_ends(&g, span, self.routing_plan.route(route), radius);
        let robots: BTreeSet<usize> = ends
            .iter()
            .flat_map(|e| e.group.iter().map(|(r, _)| *r))
            .collect();
        let exempt: BTreeSet<(usize, usize)> = ends
            .iter()
            .filter(|e| e.holds)
            .flat_map(|e| e.group.iter().copied())
            .collect();
        let name = |(r, l): (usize, usize)| {
            (
                self.robots()[r].name.clone(),
                self.robots()[r].model.links[l].name.clone(),
            )
        };
        let mut collide = Vec::new();
        for r in robots {
            for l in 0..self.robots()[r].model.links.len() {
                if !exempt.contains(&(r, l)) && !self.robots()[r].collider.link_parts(l).is_empty()
                {
                    collide.push(name((r, l)));
                }
            }
        }
        Ok((collide, exempt.into_iter().map(name).collect()))
    }

    /// Which of `links` (robot, link) any of `points` comes within `radius`
    /// of at the scene's current pose — a starting line a rope would begin
    /// inside.
    pub fn points_near_links(
        &self,
        points: &[[f64; 3]],
        radius: f64,
        links: &[(String, String)],
    ) -> Vec<(String, String)> {
        let mut out = Vec::new();
        let mut poses: BTreeMap<usize, Vec<Isometry3<f64>>> = BTreeMap::new();
        for (robot, link) in links {
            let Some(r) = self.robot_index(robot) else {
                continue;
            };
            let Some(l) = self.robots()[r].model.link_index(link) else {
                continue;
            };
            let poses = poses.entry(r).or_insert_with(|| self.link_poses_for(r));
            if points.iter().any(|p| {
                self.link_point_distance(r, l, poses, &Point3::from(*p))
                    .is_some_and(|d| d <= radius + 1e-6)
            }) {
                out.push((robot.clone(), link.clone()));
            }
        }
        out
    }

    /// The nearest body to a run of line segments of radius `radius`:
    /// enabled obstacles (less `skip_obstacle`), and every robot at its
    /// current configuration (less the links in `skip_links`), each link
    /// named. `near` limits the obstacles scanned to those whose bounds
    /// meet the box.
    fn nearest_body(
        &self,
        segments: &[(Point3<f64>, Point3<f64>)],
        radius: f64,
        skip_obstacle: &dyn Fn(&str) -> bool,
        skip_links: &BTreeSet<(usize, usize)>,
        near: Option<([f64; 3], [f64; 3])>,
    ) -> Option<Nearest> {
        let capsules: Vec<(Point3<f64>, Point3<f64>, crate::ObstacleCollider)> = segments
            .iter()
            .map(|(a, b)| (*a, *b, crate::ObstacleCollider::capsule(*a, *b, radius)))
            .collect();
        if capsules.is_empty() {
            return None;
        }
        let mut nearest: Option<Nearest> = None;
        let mut consider = |distance: f64, body: String, point: [f64; 3]| {
            if nearest.as_ref().is_none_or(|n| distance < n.distance) {
                nearest = Some(Nearest {
                    distance,
                    body,
                    point,
                    own: false,
                });
            }
        };
        for obstacle in self
            .obstacles()
            .iter()
            .filter(|o| o.enabled && !skip_obstacle(&o.name))
        {
            if let Some((lo, hi)) = near {
                if let Some((olo, ohi)) = self.obstacle_bounds(&obstacle.name) {
                    if (0..3).any(|k| ohi[k] < lo[k] || olo[k] > hi[k]) {
                        continue;
                    }
                }
            }
            let Some((_, collider)) = self.obstacle_with_collider(&obstacle.name) else {
                continue;
            };
            let world = botrail_collide::to_parry_pose(&obstacle.pose);
            for (a, b, capsule) in &capsules {
                for (cap_pose, cap_shape) in capsule.parts() {
                    for (part_pose, part_shape) in collider.parts() {
                        let Ok(d) = parry3d_f64::query::distance(
                            cap_pose,
                            cap_shape.as_ref(),
                            &(world * *part_pose),
                            part_shape.as_ref(),
                        ) else {
                            continue;
                        };
                        consider(
                            d,
                            format!("`{}`", obstacle.name),
                            to_array(nalgebra::center(a, b)),
                        );
                    }
                }
            }
        }
        // The robots, link by link: the distance to each link's parts,
        // named, pairs farther apart than LINK_PRUNE_M by their bounds
        // skipped (a finding needs CLEARANCE_MARGIN_M; the nearest body
        // beyond that is a courtesy).
        let boxes: Vec<parry3d_f64::bounding_volume::Aabb> = capsules
            .iter()
            .flat_map(|(_, _, c)| {
                c.parts()
                    .iter()
                    .map(|(pose, shape)| shape.compute_aabb(pose))
            })
            .collect();
        let apart = |a: &parry3d_f64::bounding_volume::Aabb,
                     b: &parry3d_f64::bounding_volume::Aabb| {
            a.mins.x - b.maxs.x > LINK_PRUNE_M
                || b.mins.x - a.maxs.x > LINK_PRUNE_M
                || a.mins.y - b.maxs.y > LINK_PRUNE_M
                || b.mins.y - a.maxs.y > LINK_PRUNE_M
                || a.mins.z - b.maxs.z > LINK_PRUNE_M
                || b.mins.z - a.maxs.z > LINK_PRUNE_M
        };
        for (r, robot) in self.robots().iter().enumerate() {
            let mut poses: Option<Vec<Isometry3<f64>>> = None;
            for l in 0..robot.model.links.len() {
                if skip_links.contains(&(r, l)) {
                    continue;
                }
                let parts = robot.collider.link_parts(l);
                if parts.is_empty() {
                    continue;
                }
                let poses = poses.get_or_insert_with(|| self.link_poses_for(r));
                let world = botrail_collide::to_parry_pose(&poses[l]);
                let label = format!(
                    "the robot link `{}/{}`",
                    robot.name, robot.model.links[l].name
                );
                for (part_pose, part_shape) in parts {
                    let part_world = world * *part_pose;
                    let part_box = part_shape.compute_aabb(&part_world);
                    let mut k = 0usize;
                    for (a, b, capsule) in &capsules {
                        for (cap_pose, cap_shape) in capsule.parts() {
                            let cap_box = &boxes[k];
                            k += 1;
                            if apart(cap_box, &part_box) {
                                continue;
                            }
                            if let Ok(d) = parry3d_f64::query::distance(
                                cap_pose,
                                cap_shape.as_ref(),
                                &part_world,
                                part_shape.as_ref(),
                            ) {
                                consider(d, label.clone(), to_array(nalgebra::center(a, b)));
                            }
                        }
                    }
                }
            }
        }
        nearest
    }

    /// The attachment behind an anchor of a resolved route: a port's
    /// geometry or a clamp's.
    fn anchor_attach(&self, anchor: &RouteAnchor) -> Option<Attach> {
        if let Some(at) = &anchor.attach {
            return Some(at.clone());
        }
        match anchor.kind {
            AnchorKind::Port => self
                .connection_plan()
                .ports
                .iter()
                .find(|p| p.name == anchor.name)
                .and_then(|p| p.geometry.as_ref())
                .map(|g| g.at.clone()),
            AnchorKind::Clamp => match self.routing_plan.support(&anchor.name) {
                Some(Support::Clamp { at, .. }) => Some(at.clone()),
                _ => {
                    let (carrier, k) = Support::holder_of(&anchor.name)?;
                    match self.routing_plan.support(carrier) {
                        Some(Support::Carrier { holders, .. }) => {
                            holders.get(k).map(|h| h.at.clone())
                        }
                        _ => None,
                    }
                }
            },
        }
    }

    /// The moving spans over a baked cycle: what the kinematics say
    /// (the distance between a span's holders over time — hence the free
    /// length it needs and the stroke a retraction must take up — and the
    /// twist between them), and, where the cycle carries a rope replay of
    /// the span (`bt.routing.animate`, tracks named `route:<route>#<span>`),
    /// what the replay says: the tightest bend, the stretch, the sag, the
    /// nearest body. Every finding has its time. The rope's stiffness is
    /// the solver's native setting: shape, clearance and slack are judged,
    /// tension, torsion and life are not.
    pub fn routing_timeline_report(
        &self,
        timeline: &crate::rollout::SequenceTimeline,
        dt: f64,
        sag_limit_m: f64,
    ) -> Result<RoutingTimelineReport, String> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err("dt must be positive".to_string());
        }
        if timeline.robots.len() != self.robots().len()
            || timeline
                .robots
                .iter()
                .zip(self.robots())
                .any(|(t, r)| t.name != r.name)
        {
            return Err(
                "the timeline's robots do not match the scene's (bake the cycle on this cell)"
                    .to_string(),
            );
        }
        let plan = &self.routing_plan;
        let mut world = self.clone();
        let mut span_at: Vec<Option<usize>> = vec![None; timeline.objects.len()];
        crate::verify::apply_state(&mut world, timeline, 0.0, &mut span_at)
            .map_err(|e| e.to_string())?;
        // The spans at time zero, with the attachments behind their anchors.
        struct Watched {
            route: String,
            spec: Option<CableSpec>,
            index: usize,
            span: Span,
            attaches: (Attach, Attach),
            material_m: f64,
            ducts: Vec<String>,
            skip_obstacles: BTreeSet<String>,
            /// How the span's ends are held (`HeldEnd`): which robot links
            /// are its mount, which are judged away from its ends.
            ends: [HeldEnd; 2],
            /// The link group a dressed span is laid along.
            dressed: Option<Vec<(usize, usize)>>,
            /// A fixed span's design run, scanned against the robots over
            /// the cycle (a moving span's shape is the replay's).
            points: Vec<[f64; 3]>,
            radius: f64,
            /// The first and last span of the stretch this span is replayed
            /// in (itself, unless the line slides through a holder).
            stretch: (usize, usize),
            /// The holders' freedom to roll at either end, with the line's
            /// direction there in the holder body's frame: a rolling end
            /// takes none of the holders' relative turn about it as twist.
            roll: [Option<Vector3<f64>>; 2],
            /// A retracting holder at either end: its rail's axis in the
            /// body's frame and its stroke — the other holder's distance is
            /// taken to the rail, where the carriage can go.
            rails: [Option<(Vector3<f64>, f64)>; 2],
        }
        let mut watched: Vec<Watched> = Vec::new();
        for route in &plan.routes {
            let Ok(g) = world.route_geometry(&route.name) else {
                continue;
            };
            let moving = g
                .spans
                .iter()
                .filter(|s| s.kind == SpanKind::Moving)
                .count();
            let mut moving_seen = 0usize;
            for (index, span) in g.spans.iter().enumerate() {
                let slack_share = if span.kind == SpanKind::Moving {
                    let share = route.slack_of_moving(moving_seen, moving);
                    moving_seen += 1;
                    share
                } else {
                    0.0
                };
                let end = |name: &str| {
                    let anchor = g.anchors.iter().find(|x| x.name == name)?;
                    let at = world.anchor_attach(anchor)?;
                    Some((anchor, at))
                };
                let (Some((anchor_a, a)), Some((anchor_b, b))) = (end(&span.from), end(&span.to))
                else {
                    continue;
                };
                // A holder the line rolls in (a declared rotating, sliding
                // or strap holder): the line's direction there, in the
                // body's frame.
                let roll_axis = |anchor: &RouteAnchor, at: &Attach| -> Option<Vector3<f64>> {
                    if anchor.kind != AnchorKind::Clamp || !anchor.holder.rolls() {
                        return None;
                    }
                    let t = Vector3::from(anchor.tangent?);
                    let (_, pose, _) = world.attach_point(at).ok()?;
                    Some(pose.rotation.inverse() * t)
                };
                let roll = [roll_axis(anchor_a, &a), roll_axis(anchor_b, &b)];
                let rail_of = |anchor: &RouteAnchor, at: &Attach| -> Option<(Vector3<f64>, f64)> {
                    let rail = anchor.rail?;
                    let (_, pose, _) = world.attach_point(at).ok()?;
                    Some((
                        pose.rotation.inverse() * Vector3::from(rail.axis),
                        rail.stroke_m,
                    ))
                };
                let rails = [rail_of(anchor_a, &a), rail_of(anchor_b, &b)];
                let mut skip_obstacles = BTreeSet::new();
                for at in [&a, &b] {
                    if let Attach::Obstacle { name, .. } = at {
                        skip_obstacles.insert(name.clone());
                    }
                }
                // The robot links: a holding link whose shape holds the
                // held point is the line's mount; any other link — the
                // rest of the arm a span rides included — is a body the
                // line can run into, its own holding link away from the
                // clip.
                let radius = g.od_m.map(|d| d / 2.0).unwrap_or(0.0);
                let (ends, dressed) = world.span_ends(&g, span, Some(route), radius);
                watched.push(Watched {
                    route: route.name.clone(),
                    spec: plan.spec(&route.spec).cloned(),
                    index,
                    span: span.clone(),
                    attaches: (a, b),
                    material_m: (span.to_s - span.from_s) + slack_share,
                    ducts: g.ducts.clone(),
                    skip_obstacles,
                    ends,
                    dressed,
                    points: if span.kind == SpanKind::Fixed {
                        g.span_points(span)
                    } else {
                        Vec::new()
                    },
                    radius,
                    stretch: (stretch_start(&g.spans, index), stretch_end(&g.spans, index)),
                    roll,
                    rails,
                });
            }
        }
        let mut rows: Vec<SpanMotion> = Vec::new();
        let mut findings: Vec<RoutingFinding> = Vec::new();
        let finding = |severity: &str,
                       code: &str,
                       route: &str,
                       message: String,
                       position: Option<[f64; 3]>,
                       t_s: f64,
                       findings: &mut Vec<RoutingFinding>| {
            findings.push(RoutingFinding {
                severity: severity.to_string(),
                code: code.to_string(),
                route: route.to_string(),
                message,
                position,
                s_m: None,
                t_s: Some(t_s),
                rope: None,
                sample: None,
            });
        };
        // A finding read off a replayed rope: at a sample of it, which the
        // studio follows as the cycle plays.
        let on_rope = |severity: &str,
                       code: &str,
                       route: &str,
                       message: String,
                       rope: &str,
                       sample: usize,
                       position: [f64; 3],
                       t_s: f64,
                       findings: &mut Vec<RoutingFinding>| {
            findings.push(RoutingFinding {
                severity: severity.to_string(),
                code: code.to_string(),
                route: route.to_string(),
                message,
                position: Some(position),
                s_m: None,
                t_s: Some(t_s),
                rope: Some(rope.to_string()),
                sample: Some(sample),
            });
        };
        if watched.is_empty() {
            return Ok(RoutingTimelineReport {
                spans: rows,
                findings,
            });
        }

        // ---- Kinematics: the holders' distance and relative twist over time.
        struct Kin {
            min: (f64, f64),
            max: (f64, f64),
            twist: (f64, f64),
            rel0: Option<nalgebra::UnitQuaternion<f64>>,
            /// A fixed span's nearest robot link over the cycle.
            clear: Option<(f64, f64, String, [f64; 3])>,
        }
        let mut kin: Vec<Kin> = watched
            .iter()
            .map(|_| Kin {
                min: (f64::INFINITY, 0.0),
                max: (0.0, 0.0),
                twist: (0.0, 0.0),
                rel0: None,
                clear: None,
            })
            .collect();
        let no_obstacles = |_: &str| true;
        // The holders' distances summed over a stretch (spans the line
        // slides between), at their largest.
        let mut stretch_max: BTreeMap<(String, usize), (f64, f64)> = BTreeMap::new();
        let steps = (timeline.duration / dt).ceil().max(1.0) as usize;
        for k in 0..=steps {
            let t = (k as f64 * dt).min(timeline.duration);
            crate::verify::apply_state(&mut world, timeline, t, &mut span_at)
                .map_err(|e| e.to_string())?;
            let mut stretch_sum: BTreeMap<(String, usize), f64> = BTreeMap::new();
            for (w, state) in watched.iter().zip(kin.iter_mut()) {
                let (Ok((pa, pose_a, _)), Ok((pb, pose_b, _))) = (
                    world.attach_point(&w.attaches.0),
                    world.attach_point(&w.attaches.1),
                ) else {
                    continue;
                };
                // The holders' distance — to the rail a retracting holder's
                // carriage runs on, where it can go to meet the line.
                let to_rail = |p: Point3<f64>,
                               base: Point3<f64>,
                               pose: &Isometry3<f64>,
                               rail: &(Vector3<f64>, f64)| {
                    let axis = pose.rotation * rail.0;
                    let along = (p - base).dot(&axis).clamp(0.0, rail.1);
                    (p - (base + axis * along)).norm()
                };
                let d = match (&w.rails[0], &w.rails[1]) {
                    (Some(rail), _) => to_rail(pb, pa, &pose_a, rail),
                    (None, Some(rail)) => to_rail(pa, pb, &pose_b, rail),
                    (None, None) => (pb - pa).norm(),
                };
                if d < state.min.0 {
                    state.min = (d, t);
                }
                if d > state.max.0 {
                    state.max = (d, t);
                }
                if w.stretch.1 > w.stretch.0 {
                    *stretch_sum
                        .entry((w.route.clone(), w.stretch.0))
                        .or_insert(0.0) += d;
                }
                let rel = pose_a.rotation.inverse() * pose_b.rotation;
                // The holders' whole relative rotation — bend and torsion
                // together, the worst case for the torsion rating. Taking
                // the bend out (the least rotation from the tangent leaving
                // one holder to the tangent reaching the other) is not
                // stable: a port that sweeps round to face the way it came
                // leaves the bend's plane undetermined and the remainder
                // reads as a large twist (design-routing.md §15.13). The
                // true twist is a frame carried along the line's shape,
                // which the replay has and the kinematics do not.
                // A holder the line rolls in takes none of the turn about
                // the line's direction there: that part of the relative
                // rotation is left out (the swing of a swing–twist split).
                let twist = match state.rel0 {
                    None => {
                        state.rel0 = Some(rel);
                        0.0
                    }
                    Some(rel0) => {
                        let q = rel * rel0.inverse();
                        let mut axes: Vec<Vector3<f64>> = Vec::new();
                        if let Some(tb) = w.roll[1] {
                            axes.push(rel * tb);
                        }
                        if let Some(ta) = w.roll[0] {
                            axes.push(ta);
                        }
                        swing_angle(&q, &axes).to_degrees()
                    }
                };
                if twist > state.twist.0 {
                    state.twist = (twist, t);
                }
                // The design run of a fixed span against the arms as they
                // move: the robot hitting a fixed cable, with its time.
                if w.points.len() >= 2 {
                    if let Some(n) = world.nearest_body_held(
                        &w.points,
                        w.radius,
                        &no_obstacles,
                        &w.ends,
                        w.dressed.as_ref(),
                        None,
                    ) {
                        if state.clear.as_ref().is_none_or(|c| n.distance < c.0) {
                            state.clear = Some((n.distance, t, n.body, n.point));
                        }
                    }
                }
            }
            for (key, sum) in stretch_sum {
                let best = stretch_max.entry(key).or_insert((0.0, 0.0));
                if sum > best.0 {
                    *best = (sum, t);
                }
            }
        }

        let mut seen_ropes: BTreeSet<String> = BTreeSet::new();
        for (w, state) in watched.iter().zip(&kin) {
            // A span inside a carrier answers to the hose as well: the
            // larger bend radius, the smaller torsion tolerance.
            let carrier_span = carrier_span_of(&w.span);
            let hose = carrier_span
                .as_ref()
                .and_then(|(c, _)| match plan.support(c) {
                    Some(Support::Carrier {
                        bend_r_moving_m,
                        torsion_deg_m,
                        ..
                    }) => Some((*bend_r_moving_m, *torsion_deg_m)),
                    _ => None,
                });
            let bend_moving = match (
                w.spec.as_ref().and_then(|s| s.bend_r_moving_m),
                hose.and_then(|h| h.0),
            ) {
                (Some(a), Some(b)) => Some(a.max(b)),
                (a, b) => a.or(b),
            };
            let torsion = match (
                w.spec.as_ref().and_then(|s| s.torsion_deg_m),
                hose.and_then(|h| h.1),
            ) {
                (Some(a), Some(b)) => Some(a.min(b)),
                (a, b) => a.or(b),
            };
            // A dressed span rides its body: the holders' distance is the
            // line between them, no bend allowance to add.
            let required = state.max.0
                + if w.span.kind == SpanKind::Moving {
                    2.0 * bend_moving.unwrap_or(0.0)
                } else {
                    0.0
                };
            let stroke = (state.max.0 - state.min.0).max(0.0);
            let mut row = SpanMotion {
                route: w.route.clone(),
                span: w.index,
                from: w.span.from.clone(),
                to: w.span.to.clone(),
                kind: w.span.kind,
                material_m: round_m(w.material_m),
                min_distance_m: round_m(state.min.0),
                max_distance_m: round_m(state.max.0),
                stroke_m: round_m(stroke),
                required_free_m: round_m(required),
                twist_deg: (state.twist.0 * 10.0).round() / 10.0,
                rope: None,
                min_bend_radius_m: None,
                max_stretch: None,
                max_sag_m: None,
                retraction_m: None,
                torsion_deg: None,
                min_clearance_m: None,
                nearest: None,
                status: "ok".to_string(),
            };
            let before = findings.len();
            if let Some((d, t, body, at)) = state
                .clear
                .as_ref()
                .filter(|_| w.span.kind == SpanKind::Fixed)
            {
                row.min_clearance_m = Some(round_m(*d));
                row.nearest = Some(body.clone());
                if *d <= 1e-6 {
                    finding(
                        "error",
                        "route_contact",
                        &w.route,
                        format!(
                            "{body} runs into the fixed span {} → {} at t = {t:.2} s",
                            w.span.from, w.span.to
                        ),
                        Some(*at),
                        *t,
                        &mut findings,
                    );
                } else if *d < CLEARANCE_MARGIN_M {
                    finding(
                        "warning",
                        "route_clearance",
                        &w.route,
                        format!(
                            "{body} passes {:.0} mm from the fixed span {} → {} at t = {t:.2} s",
                            d * 1000.0,
                            w.span.from,
                            w.span.to
                        ),
                        Some(*at),
                        *t,
                        &mut findings,
                    );
                }
            }
            let in_stretch = w.stretch.1 > w.stretch.0;
            if in_stretch && w.index == w.stretch.0 {
                // The line slides through the holders inside the stretch:
                // its material is shared, and judged over the whole.
                let members: Vec<&Watched> = watched
                    .iter()
                    .filter(|m| m.route == w.route && m.stretch == w.stretch)
                    .collect();
                let material: f64 = members.iter().map(|m| m.material_m).sum();
                let (apart, when) = stretch_max
                    .get(&(w.route.clone(), w.stretch.0))
                    .copied()
                    .unwrap_or((0.0, 0.0));
                let needed = apart + bend_moving.map_or(0.0, |r| 2.0 * r) * members.len() as f64;
                if members.iter().any(|m| m.span.kind == SpanKind::Moving)
                    && material + 1e-6 < needed
                {
                    let through: Vec<&str> =
                        members[1..].iter().map(|m| m.span.from.as_str()).collect();
                    finding(
                        "error",
                        "route_tension",
                        &w.route,
                        format!(
                            "the line from {} to {} slides through {} and has {:.2} m between them but needs {:.2} m at t = {:.2} s ({:.2} m of holder distances{}) — add slack",
                            w.span.from,
                            members.last().map(|m| m.span.to.as_str()).unwrap_or(""),
                            through.join(", "),
                            material,
                            needed,
                            when,
                            apart,
                            bend_moving.map(|r| format!(" + 2 × {:.0} mm bend radius a span", r * 1000.0)).unwrap_or_default()
                        ),
                        None,
                        when,
                        &mut findings,
                    );
                }
            }
            if w.span.kind == SpanKind::Moving && !in_stretch && w.material_m + 1e-6 < required {
                finding(
                    "error",
                    "route_tension",
                    &w.route,
                    format!(
                        "span {} → {} has {:.2} m of line between its holders but needs {:.2} m at t = {:.2} s ({:.2} m apart{}) — add slack",
                        w.span.from,
                        w.span.to,
                        w.material_m,
                        required,
                        state.max.1,
                        state.max.0,
                        bend_moving.map(|r| format!(" + 2 × {:.0} mm bend radius", r * 1000.0)).unwrap_or_default()
                    ),
                    None,
                    state.max.1,
                    &mut findings,
                );
            }
            // ---- The rope replay of this span, when the cycle carries one.
            // A span inside a carrier reads the hose's replay when the cycle
            // carries one (`carrier:<c>#<i>`), its own otherwise; the hose's
            // findings are the carrier's, reported once.
            let spans_of_route: Vec<Span> = watched
                .iter()
                .filter(|m| m.route == w.route)
                .map(|m| m.span.clone())
                .collect();
            let mut names = span_rope_names(&w.route, &spans_of_route, w.index).into_iter();
            let own_name = names.next().expect("a span's own rope name");
            let hose_name = names.next();
            let (rope_name, owner) = match hose_name {
                Some(h) if timeline.ropes.iter().any(|r| r.name == h) => (
                    h,
                    carrier_span
                        .as_ref()
                        .map(|(c, _)| c.clone())
                        .unwrap_or_default(),
                ),
                _ => (own_name, w.route.clone()),
            };
            let first_sight = seen_ropes.insert(rope_name.clone());
            // The twist the line takes, against its torsion rating: the
            // torsion a static rod carries between its holders when the
            // track has it — the kinematics' relative turn (bend and torsion
            // together, the worst case) otherwise.
            let torsion_found: Option<(f64, f64)> = timeline
                .ropes
                .iter()
                .find(|r| r.name == rope_name)
                .and_then(|r| {
                    let v = r.twist_deg.as_ref()?;
                    v.iter().zip(&r.times).map(|(a, t)| (a.abs(), *t)).fold(
                        None,
                        |m: Option<(f64, f64)>, (a, t)| match m {
                            Some(b) if b.0 >= a => Some(b),
                            _ => Some((a, t)),
                        },
                    )
                });
            row.torsion_deg = torsion_found.map(|(a, _)| (a * 10.0).round() / 10.0);
            if let Some(torsion) = torsion {
                let allowed = torsion * w.material_m;
                let (twist, when, how) = match torsion_found {
                    Some((a, t)) => (a, t, "is twisted"),
                    None => (state.twist.0, state.twist.1, "twists"),
                };
                if twist > allowed + 1e-9 {
                    finding(
                        "error",
                        "route_twist",
                        &w.route,
                        format!(
                            "span {} → {} {how} {:.0}° at t = {:.2} s; the line allows {:.0}°/m over {:.2} m = {:.0}°",
                            w.span.from, w.span.to, twist, when, torsion, w.material_m, allowed
                        ),
                        None,
                        when,
                        &mut findings,
                    );
                }
            }
            let rope_from = findings.len();
            if let Some(track) = timeline.ropes.iter().find(|r| r.name == rope_name) {
                row.rope = Some(rope_name.clone());
                if track.points.len() >= 2 && track.points[0].len() >= 3 {
                    // The rope's particle spacing, from its first frame —
                    // the median, not each gap: a rope settled with slack
                    // bunches a few particles almost onto each other, and a
                    // gap of half a millimetre opening to its usual length
                    // is not a strain.
                    let rest = {
                        let mut gaps: Vec<f64> = track.points[0]
                            .windows(2)
                            .map(|w| (Vector3::from(w[1]) - Vector3::from(w[0])).norm())
                            .collect();
                        gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                        gaps.get(gaps.len() / 2).copied().unwrap_or(0.0)
                    };
                    // A fresh replay of the world for the clearance scan.
                    let mut world2 = self.clone();
                    let mut span_at2: Vec<Option<usize>> = vec![None; timeline.objects.len()];
                    let duct_prefixes: Vec<String> =
                        w.ducts.iter().map(|d| format!("{d}/")).collect();
                    let skip = |name: &str| {
                        w.skip_obstacles.contains(name)
                            || duct_prefixes.iter().any(|p| name.starts_with(p))
                            || w.ducts.iter().any(|d| d == name)
                    };
                    // Each with the time, the place and the sample it was read at.
                    let mut min_radius: Option<(f64, f64, [f64; 3], usize)> = None;
                    let mut max_stretch: (f64, f64, [f64; 3], usize) = (0.0, 0.0, [0.0; 3], 0);
                    let mut max_sag: (f64, f64, [f64; 3], usize) = (0.0, 0.0, [0.0; 3], 0);
                    let mut min_clear: Option<(f64, f64, String, [f64; 3], bool, usize)> = None;
                    // The carriage's travel along its rail: least, most, when most.
                    let mut travel: Option<(f64, f64, f64)> = None;
                    let mut last_scanned = f64::NEG_INFINITY;
                    let last = track.times.len() - 1;
                    for (i, (&t, pts)) in track.times.iter().zip(&track.points).enumerate() {
                        if i != 0 && i != last && t - last_scanned < dt - 1e-9 {
                            continue;
                        }
                        last_scanned = t;
                        // Stretch along the particles: over a few
                        // segments at a time against the spacing, so one
                        // gap is not read as the line pulled taut.
                        const STRETCH_WIN: usize = 4;
                        if rest > EPS && pts.len() > STRETCH_WIN {
                            for j in STRETCH_WIN..pts.len() {
                                let len: f64 = (j + 1 - STRETCH_WIN..=j)
                                    .map(|i| {
                                        (Vector3::from(pts[i]) - Vector3::from(pts[i - 1])).norm()
                                    })
                                    .sum();
                                let stretch = len / (STRETCH_WIN as f64 * rest) - 1.0;
                                if stretch > max_stretch.0 {
                                    let at = j - STRETCH_WIN / 2;
                                    max_stretch = (stretch, t, pts[at], at);
                                }
                            }
                        }
                        // Bend: the turn over two particles each side (a
                        // one-particle window reads the chain's own
                        // granularity as a fold), away from the holders,
                        // where a point hold lets the particle rope fold in
                        // a way a connector's boot never does.
                        const WIN: usize = 2;
                        for j in (WIN + 1)..pts.len().saturating_sub(WIN + 1) {
                            let a = Vector3::from(pts[j]) - Vector3::from(pts[j - WIN]);
                            let b = Vector3::from(pts[j + WIN]) - Vector3::from(pts[j]);
                            let (la, lb) = (a.norm(), b.norm());
                            if la > EPS && lb > EPS {
                                let theta = (a.dot(&b) / (la * lb)).clamp(-1.0, 1.0).acos();
                                if theta > 1e-6 {
                                    let radius = ((la + lb) / 2.0) / theta;
                                    if min_radius.is_none_or(|m| radius < m.0) {
                                        min_radius = Some((radius, t, pts[j], j));
                                    }
                                }
                            }
                        }
                        // Sag: the farthest particle from the chord between the ends.
                        let p0 = Vector3::from(pts[0]);
                        let p1 = Vector3::from(pts[pts.len() - 1]);
                        let chord = p1 - p0;
                        let chord_len = chord.norm();
                        for (j, p) in pts.iter().enumerate() {
                            let v = Vector3::from(*p) - p0;
                            let sag = if chord_len > EPS {
                                let s = (v.dot(&chord) / (chord_len * chord_len)).clamp(0.0, 1.0);
                                (v - chord * s).norm()
                            } else {
                                v.norm()
                            };
                            if sag > max_sag.0 {
                                max_sag = (sag, t, *p, j);
                            }
                        }
                        // Clearance against the world at this time.
                        crate::verify::apply_state(&mut world2, timeline, t, &mut span_at2)
                            .map_err(|e| e.to_string())?;
                        // A retracting holder's carriage: where the held
                        // sample is along its rail.
                        for (end, rail) in w.rails.iter().enumerate() {
                            let Some((axis_local, _)) = rail else {
                                continue;
                            };
                            let at = if end == 0 {
                                &w.attaches.0
                            } else {
                                &w.attaches.1
                            };
                            let Ok((base, pose, _)) = world2.attach_point(at) else {
                                continue;
                            };
                            let sample = if end == 0 { pts[0] } else { pts[pts.len() - 1] };
                            let along =
                                (Point3::from(sample) - base).dot(&(pose.rotation * axis_local));
                            let travel = travel.get_or_insert((along, along, t));
                            if along < travel.0 {
                                travel.0 = along;
                            }
                            if along > travel.1 {
                                travel.1 = along;
                                travel.2 = t;
                            }
                        }
                        let mut lo = [f64::INFINITY; 3];
                        let mut hi = [f64::NEG_INFINITY; 3];
                        for p in pts {
                            for k in 0..3 {
                                lo[k] = lo[k].min(p[k] - track.radius_m - 0.05);
                                hi[k] = hi[k].max(p[k] + track.radius_m + 0.05);
                            }
                        }
                        if let Some(n) = world2.nearest_body_held(
                            pts,
                            track.radius_m,
                            &skip,
                            &w.ends,
                            w.dressed.as_ref(),
                            Some((lo, hi)),
                        ) {
                            if min_clear.as_ref().is_none_or(|m| n.distance < m.0) {
                                let at = nearest_sample(pts, n.point);
                                min_clear = Some((n.distance, t, n.body, n.point, n.own, at));
                            }
                        }
                    }
                    row.min_bend_radius_m = min_radius.map(|m| round_m(m.0));
                    row.max_stretch = Some((max_stretch.0 * 1000.0).round() / 1000.0);
                    row.max_sag_m = (w.span.kind != SpanKind::Fixed).then(|| round_m(max_sag.0));
                    // The retraction's carriage: how far it ran, and whether
                    // it ran out of rail.
                    if let Some((least, most, when)) = travel {
                        row.retraction_m = Some(round_m((most - least).max(0.0)));
                        let stroke = w.rails.iter().flatten().map(|r| r.1).next().unwrap_or(0.0);
                        if most >= stroke - 1e-3 && stroke > 0.0 {
                            finding(
                                "warning",
                                "route_stroke",
                                &owner,
                                format!(
                                    "span {} → {}: the retraction runs out of its {:.2} m stroke at t = {:.2} s — a longer rail, or more line",
                                    w.span.from, w.span.to, stroke, when
                                ),
                                None,
                                when,
                                &mut findings,
                            );
                        }
                    }
                    row.min_clearance_m = min_clear.as_ref().map(|m| round_m(m.0));
                    row.nearest = min_clear.as_ref().map(|m| m.2.clone());
                    if let (Some(limit), Some((radius, t, at, j))) = (bend_moving, min_radius) {
                        if w.span.kind != SpanKind::Fixed && radius < limit {
                            on_rope(
                                "error",
                                "route_bend_moving",
                                &owner,
                                format!(
                                    "span {} → {} bends to r = {:.0} mm at t = {:.2} s; the line allows r ≥ {:.0} mm in motion",
                                    w.span.from, w.span.to, radius * 1000.0, t, limit * 1000.0
                                ),
                                &rope_name,
j,
at,
t,
&mut findings,
                            );
                        }
                    }
                    if w.span.kind != SpanKind::Fixed && max_stretch.0 > 0.25 {
                        on_rope(
                            "warning",
                            "route_stretch",
                            &owner,
                            format!(
                                "span {} → {} strains {:.0} % at t = {:.2} s in the replay — nearly taut between its holders (the rope is soft: the free length is the figure to act on)",
                                w.span.from, w.span.to, max_stretch.0 * 100.0, max_stretch.1
                            ),
                            &rope_name,
max_stretch.3,
max_stretch.2,
max_stretch.1,
&mut findings,
                        );
                    }
                    if w.span.kind != SpanKind::Fixed && max_sag.0 > sag_limit_m {
                        on_rope(
                            "warning",
                            "route_sag",
                            &owner,
                            format!(
                                "span {} → {} hangs {:.0} mm off its chord at t = {:.2} s (limit {:.0} mm) — too much slack, or a clamp missing",
                                w.span.from, w.span.to, max_sag.0 * 1000.0, max_sag.1, sag_limit_m * 1000.0
                            ),
                            &rope_name,
max_sag.3,
max_sag.2,
max_sag.1,
&mut findings,
                        );
                    }
                    // A fixed span hung under gravity rests on what is
                    // under it; only a span that moves is reported for
                    // touching. Lying along the link it is tied to (a stiff
                    // line leaves its clip along the arm) rubs only as far
                    // as its other end moves: a warning; any other body an
                    // error.
                    if let Some((d, t, body, at, own, j)) = min_clear
                        .as_ref()
                        .filter(|_| w.span.kind != SpanKind::Fixed)
                    {
                        if *d <= 1e-6 && *own {
                            on_rope(
                                "warning",
                                "route_contact",
                                &owner,
                                format!(
                                    "span {} → {} lies on {body}, which holds one end of it, at t = {t:.2} s",
                                    w.span.from, w.span.to
                                ),
                                &rope_name,
*j,
*at,
*t,
&mut findings,
                            );
                        } else if *d <= 1e-6 {
                            on_rope(
                                "error",
                                "route_contact",
                                &owner,
                                format!(
                                    "span {} → {} touches {body} at t = {t:.2} s",
                                    w.span.from, w.span.to
                                ),
                                &rope_name,
                                *j,
                                *at,
                                *t,
                                &mut findings,
                            );
                        } else if *d < CLEARANCE_MARGIN_M {
                            on_rope(
                                "warning",
                                "route_clearance",
                                &owner,
                                format!(
                                    "span {} → {} passes {:.0} mm from {body} at t = {t:.2} s",
                                    w.span.from,
                                    w.span.to,
                                    d * 1000.0
                                ),
                                &rope_name,
                                *j,
                                *at,
                                *t,
                                &mut findings,
                            );
                        }
                    }
                }
            }
            let severity_of = |fs: &[RoutingFinding]| -> &'static str {
                if fs.iter().any(|f| f.severity == "error") {
                    "error"
                } else if fs.iter().any(|f| f.severity == "warning") {
                    "warning"
                } else {
                    "ok"
                }
            };
            let rope_findings: Vec<RoutingFinding> = findings.drain(rope_from..).collect();
            let mut status = severity_of(&findings[before..]);
            let rope_status = severity_of(&rope_findings);
            if rope_status == "error" || (rope_status == "warning" && status == "ok") {
                status = rope_status;
            }
            if first_sight {
                findings.extend(rope_findings);
            }
            row.status = status.to_string();
            rows.push(row);
        }
        // Lines through one another: two replayed lines whose centrelines
        // come closer than their radii (a millimetre short of touching) —
        // the replay's ropes do not see each other; the static solve keeps
        // lines apart, and a crossing it reports is one its laying could
        // not undo.
        let tracks: Vec<&RopeTrack> = timeline
            .ropes
            .iter()
            .filter(|r| r.name.starts_with("route:") || r.name.starts_with("carrier:"))
            .collect();
        let owner = |name: &str| -> String {
            let body = name.split_once(':').map(|(_, b)| b).unwrap_or(name);
            body.rsplit_once('#')
                .map(|(o, _)| o)
                .unwrap_or(body)
                .to_string()
        };
        // The holders a track ends at: two tracks that share one meet there
        // by design (a span and the hose it leads into).
        let mut ends: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        for row in &rows {
            if let Some(rope) = &row.rope {
                let e = ends.entry(rope.clone()).or_default();
                e.insert(row.from.clone());
                e.insert(row.to.clone());
            }
        }
        for i in 0..tracks.len() {
            for j in i + 1..tracks.len() {
                let (a, b) = (tracks[i], tracks[j]);
                if owner(&a.name) == owner(&b.name) {
                    continue;
                }
                if let (Some(ea), Some(eb)) = (ends.get(&a.name), ends.get(&b.name)) {
                    if ea.intersection(eb).next().is_some() {
                        continue;
                    }
                }
                let need = a.radius_m + b.radius_m - 1e-3;
                let mut worst: Option<(f64, f64)> = None;
                // At the tracks' own frames: a blend between two frames of
                // lines passing each other would meet where neither did.
                let mut when: Vec<f64> = a.times.iter().chain(&b.times).copied().collect();
                when.sort_by(f64::total_cmp);
                when.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
                for t in when {
                    let pa = a.positions_at(t);
                    let pb = b.positions_at(t);
                    for sa in pa.windows(2) {
                        for sb in pb.windows(2) {
                            let d = segment_distance(
                                Point3::from(sa[0]),
                                Point3::from(sa[1]),
                                Point3::from(sb[0]),
                                Point3::from(sb[1]),
                            );
                            if d < need && worst.is_none_or(|w| d < w.0) {
                                worst = Some((d, t));
                            }
                        }
                    }
                }
                if let Some((d, t)) = worst {
                    // Named for a route before a carrier.
                    let route = if a.name.starts_with("route:") {
                        owner(&a.name)
                    } else {
                        owner(&b.name)
                    };
                    finding(
                        "warning",
                        "route_crossing",
                        &route,
                        format!(
                            "{} passes through {} at t = {t:.2} s ({:.0} mm between their centrelines, {:.0} mm would touch)",
                            a.name,
                            b.name,
                            d * 1000.0,
                            (a.radius_m + b.radius_m) * 1000.0
                        ),
                        None,
                        t,
                        &mut findings,
                    );
                }
            }
        }
        Ok(RoutingTimelineReport {
            spans: rows,
            findings,
        })
    }
}

/// The nearest body to a run of line, as [`Scene::nearest_body`] finds it.
struct Nearest {
    distance: f64,
    body: String,
    point: [f64; 3],
    /// The body is a link of the group one of the span's ends is held on:
    /// the line lies along the arm it is tied to.
    own: bool,
}

/// One moving span over a baked cycle (and any span the cycle carries a
/// rope replay of).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SpanMotion {
    pub route: String,
    pub span: usize,
    pub from: String,
    pub to: String,
    pub kind: SpanKind,
    /// Line between the holders: the span's run plus its share of the slack.
    pub material_m: f64,
    pub min_distance_m: f64,
    pub max_distance_m: f64,
    /// What a retraction has to take up.
    pub stroke_m: f64,
    /// The farthest the holders get, plus a bend radius at each end.
    pub required_free_m: f64,
    /// The largest relative rotation of the holders about each other.
    pub twist_deg: f64,
    /// The rope track of the replay, when the cycle carries one.
    pub rope: Option<String>,
    pub min_bend_radius_m: Option<f64>,
    pub max_stretch: Option<f64>,
    pub max_sag_m: Option<f64>,
    /// How far a retracting holder's carriage ran along its rail over the
    /// cycle (m), from the replay.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retraction_m: Option<f64>,
    /// The largest torsion the static rod carried between the holders
    /// (degrees), when the track has it; `twist_deg` is the holders'
    /// relative turn, bend and torsion together.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub torsion_deg: Option<f64>,
    pub min_clearance_m: Option<f64>,
    pub nearest: Option<String>,
    pub status: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RoutingTimelineReport {
    pub spans: Vec<SpanMotion>,
    pub findings: Vec<RoutingFinding>,
}

// ------------------------------------------------------------ candidates

/// One way a route could run: the ducts it would take, with the way
/// points the author fixed kept in place, and what that way measures.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct RouteCandidate {
    pub via: Vec<Via>,
    /// The ducts passed, in order.
    pub ducts: Vec<String>,
    pub length_m: f64,
    pub cut_length_m: f64,
    /// The run outside any duct (m): a cable lying open on the floor.
    pub open_m: f64,
    pub bends: usize,
    /// Supports the way passes: ducts, clamps, carriers, auto clamps.
    pub supports: usize,
    /// Static findings on the route (and on the ducts it would share).
    pub errors: usize,
    pub warnings: usize,
    pub codes: Vec<String>,
    /// Length + the open run again + 0.3 per bend + 0.2 per support,
    /// metres — a metre in a duct beats a metre on the floor; what the
    /// list is ordered by after the error count.
    pub score: f64,
    /// This is the way the route takes now.
    pub current: bool,
}

/// Every merge of two sequences that keeps each one's order.
fn merges<T: Clone>(a: &[T], b: &[T]) -> Vec<Vec<T>> {
    if a.is_empty() {
        return vec![b.to_vec()];
    }
    if b.is_empty() {
        return vec![a.to_vec()];
    }
    let mut out = Vec::new();
    for mut rest in merges(&a[1..], b) {
        rest.insert(0, a[0].clone());
        out.push(rest);
    }
    for mut rest in merges(a, &b[1..]) {
        rest.insert(0, b[0].clone());
        out.push(rest);
    }
    out
}

/// Ordered selections of up to `k` of `items`, the empty one first.
fn sequences_up_to(items: &[String], k: usize) -> Vec<Vec<String>> {
    let mut out: Vec<Vec<String>> = vec![Vec::new()];
    let mut frontier: Vec<Vec<String>> = vec![Vec::new()];
    for _ in 0..k.min(items.len()) {
        let mut next = Vec::new();
        for seq in &frontier {
            for item in items {
                if !seq.contains(item) {
                    let mut longer = seq.clone();
                    longer.push(item.clone());
                    next.push(longer);
                }
            }
        }
        out.extend(next.iter().cloned());
        frontier = next;
    }
    out
}

/// How many ducts a candidate may pass.
const CANDIDATE_DUCTS_MAX: usize = 3;

impl Scene {
    /// The ways a route could take through the ducts the cell has: the way
    /// points the author fixed (points, clamps, carriers, runs along a
    /// body) stay in order, and every ordered choice of up to three ducts
    /// is tried between them. Each candidate is resolved and checked
    /// statically at the current poses; the list is ordered by error
    /// count, then by `score` (length + 0.3 per bend + 0.2 per support),
    /// then warnings — the same order every time for the same cell.
    pub fn route_candidates(&self, name: &str) -> Result<Vec<RouteCandidate>, String> {
        let plan = &self.routing_plan;
        let (index, route) = plan
            .routes
            .iter()
            .enumerate()
            .find(|(_, r)| r.name == name)
            .ok_or_else(|| format!("unknown route `{name}`"))?;
        let ducts: Vec<String> = plan
            .supports
            .iter()
            .filter(|s| matches!(s, Support::Duct { .. }))
            .map(|s| s.name().to_string())
            .collect();
        let fixed: Vec<Via> = route
            .via
            .iter()
            .filter(|v| !matches!(v, Via::Support { name } if ducts.contains(name)))
            .cloned()
            .collect();
        let mut candidates: Vec<RouteCandidate> = Vec::new();
        let mut seen: BTreeSet<String> = BTreeSet::new();
        for sequence in sequences_up_to(&ducts, CANDIDATE_DUCTS_MAX) {
            let ducts_as_via: Vec<Via> = sequence
                .iter()
                .map(|d| Via::Support { name: d.clone() })
                .collect();
            for via in merges(&fixed, &ducts_as_via) {
                let key = serde_json::to_string(&via).map_err(|e| e.to_string())?;
                if !seen.insert(key) {
                    continue;
                }
                let mut world = self.clone();
                world.routing_plan.routes[index].via = via.clone();
                let report = world.routing_report();
                let mine: Vec<&RoutingFinding> = report
                    .findings
                    .iter()
                    .filter(|f| {
                        f.route == name
                            || (matches!(f.code.as_str(), "duct_fill" | "route_separation")
                                && sequence.contains(&f.route))
                    })
                    .collect();
                let errors = mine.iter().filter(|f| f.severity == "error").count();
                let warnings = mine.iter().filter(|f| f.severity == "warning").count();
                let codes: Vec<String> = mine
                    .iter()
                    .map(|f| f.code.clone())
                    .collect::<BTreeSet<_>>()
                    .into_iter()
                    .collect();
                let (length_m, cut_length_m, open_m, bends, supports) =
                    match &report.geometries[index] {
                        Some(g) => (
                            g.length_m,
                            g.cut_length_m,
                            (g.length_m - g.ducted_m).max(0.0),
                            g.bends.len(),
                            g.ducts.len() + g.clamps.len() + g.carriers.len() + g.auto_clamps.len(),
                        ),
                        None => (f64::INFINITY, f64::INFINITY, f64::INFINITY, 0, 0),
                    };
                let score = length_m + open_m + 0.3 * bends as f64 + 0.2 * supports as f64;
                candidates.push(RouteCandidate {
                    current: via == route.via,
                    via,
                    ducts: sequence.clone(),
                    length_m,
                    cut_length_m,
                    open_m,
                    bends,
                    supports,
                    errors,
                    warnings,
                    codes,
                    score,
                });
            }
        }
        candidates.sort_by(|a, b| {
            a.errors
                .cmp(&b.errors)
                .then(
                    a.score
                        .partial_cmp(&b.score)
                        .unwrap_or(std::cmp::Ordering::Equal),
                )
                .then(a.warnings.cmp(&b.warnings))
                .then(a.via.len().cmp(&b.via.len()))
        });
        Ok(candidates)
    }

    /// Replaces one route's way (its `via`), keeping everything else.
    pub fn set_route_via(&mut self, name: &str, via: Vec<Via>) -> Result<(), String> {
        let mut plan = self.routing_plan.clone();
        let route = plan
            .routes
            .iter_mut()
            .find(|r| r.name == name)
            .ok_or_else(|| format!("unknown route `{name}`"))?;
        route.via = via;
        self.set_routing_plan(plan)
    }
}

// ------------------------------------------------------- planner colliders

/// A fixed span as the collision checks see it (D6): the planner, the
/// tick verification and the clearance readout treat it like an obstacle,
/// named `route:<route>#<span>`.
#[derive(Clone)]
pub struct RouteCollider {
    pub route: String,
    pub span: usize,
    pub name: String,
    pub collider: crate::ObstacleCollider,
    /// `(robot, link)` pairs allowed to touch it: the rigid groups its two
    /// anchors sit on (a connector in the arm's base).
    pub allow_links: Vec<(usize, usize)>,
}

/// The route colliders, cached under a key of everything their shape
/// derives from (the plans, the static residents' poses).
#[derive(Default)]
pub struct RouteCache(pub(crate) std::sync::RwLock<Option<(u64, Vec<RouteCollider>)>>);

impl Clone for RouteCache {
    fn clone(&self) -> Self {
        RouteCache(std::sync::RwLock::new(
            self.0.read().unwrap_or_else(|e| e.into_inner()).clone(),
        ))
    }
}

/// A read lock on the cached route colliders, held while a query uses them.
pub struct RouteColliders<'a>(
    pub(crate) std::sync::RwLockReadGuard<'a, Option<(u64, Vec<RouteCollider>)>>,
);

impl RouteColliders<'_> {
    pub fn iter(&self) -> impl Iterator<Item = &RouteCollider> {
        self.0
            .as_ref()
            .map(|c| c.1.as_slice())
            .unwrap_or(&[])
            .iter()
    }

    pub fn get(&self, k: usize) -> Option<&RouteCollider> {
        self.0.as_ref().and_then(|c| c.1.get(k))
    }

    pub fn len(&self) -> usize {
        self.0.as_ref().map(|c| c.1.len()).unwrap_or(0)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Appends the colliders to an obstacle query (world poses: identity,
    /// the capsules are in world metres).
    pub fn extend_query<'q>(
        &'q self,
        query: &mut Vec<(Isometry3<f64>, &'q crate::ObstacleCollider)>,
    ) {
        for rc in self.iter() {
            query.push((Isometry3::identity(), &rc.collider));
        }
    }

    /// The links a span's anchors sit on may touch it; `n_obstacles` is
    /// where the route colliders start in the query.
    pub fn allow_into(
        &self,
        allowance: &mut botrail_collide::ContactAllowance,
        n_obstacles: usize,
    ) {
        for (j, rc) in self.iter().enumerate() {
            for (r, l) in &rc.allow_links {
                allowance.allow(*r, *l, n_obstacles + j);
            }
        }
    }
}

impl Scene {
    /// What the route colliders derive from: the plans, every obstacle's
    /// name / pose / enabled flag, the frames, the robots' base poses and
    /// what they carry (the joints do not move a fixed span).
    fn route_cache_key(&self) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        let iso = |p: &Isometry3<f64>, h: &mut std::collections::hash_map::DefaultHasher| {
            for v in p.translation.vector.iter().chain(p.rotation.coords.iter()) {
                v.to_bits().hash(h);
            }
        };
        self.plan_gen.hash(&mut h);
        for o in self.obstacles() {
            o.name.hash(&mut h);
            o.enabled.hash(&mut h);
            iso(&o.pose, &mut h);
        }
        for f in &self.frames {
            f.name.hash(&mut h);
            iso(&f.pose, &mut h);
        }
        for r in self.robots() {
            r.name.hash(&mut h);
            iso(&r.base, &mut h);
        }
        for a in &self.attachments {
            a.object.hash(&mut h);
            a.robot.hash(&mut h);
            a.link.hash(&mut h);
        }
        h.finish()
    }

    fn build_route_colliders(&self) -> Vec<RouteCollider> {
        let mut out = Vec::new();
        for route in &self.routing_plan.routes {
            if !route.collide {
                continue;
            }
            let Ok(g) = self.route_geometry(&route.name) else {
                continue;
            };
            let radius = g.od_m.map(|d| d / 2.0).unwrap_or(0.0);
            for (k, span) in g.spans.iter().enumerate() {
                if span.kind != SpanKind::Fixed {
                    continue;
                }
                let mut parts = Vec::new();
                for w in g.span_points(span).windows(2) {
                    let (a, b) = (Point3::from(w[0]), Point3::from(w[1]));
                    if (b - a).norm() > EPS {
                        parts.extend(
                            crate::ObstacleCollider::capsule(a, b, radius)
                                .parts()
                                .iter()
                                .cloned(),
                        );
                    }
                }
                if parts.is_empty() {
                    continue;
                }
                let mut allow_links = Vec::new();
                for end in [&span.from, &span.to] {
                    let Some(anchor) = g.anchors.iter().find(|a| &a.name == end) else {
                        continue;
                    };
                    if let Some(Attach::Link { robot, link, .. }) = self.anchor_attach(anchor) {
                        if let Some(r) = self.robot_index(&robot) {
                            let model = &self.robots()[r].model;
                            if let Some(l) = model.link_index(&link) {
                                for member in rigid_group(model, l) {
                                    allow_links.push((r, member));
                                }
                            }
                        }
                    }
                }
                out.push(RouteCollider {
                    route: route.name.clone(),
                    span: k,
                    name: format!("route:{}#{k}", route.name),
                    collider: crate::ObstacleCollider::from_parts(parts),
                    allow_links,
                });
            }
        }
        out
    }

    /// The fixed spans of the `collide` routes as colliders, current for
    /// the scene as it stands (rebuilt when the key changes; a scene
    /// without routes pays a lock and nothing else).
    pub(crate) fn route_colliders(&self) -> RouteColliders<'_> {
        let read = || self.route_cache.0.read().unwrap_or_else(|e| e.into_inner());
        if self.routing_plan.routes.is_empty() {
            let guard = read();
            if guard.as_ref().is_none_or(|c| c.1.is_empty()) {
                return RouteColliders(guard);
            }
            drop(guard);
            *self
                .route_cache
                .0
                .write()
                .unwrap_or_else(|e| e.into_inner()) = None;
            return RouteColliders(read());
        }
        let key = self.route_cache_key();
        {
            let guard = read();
            if guard.as_ref().is_some_and(|c| c.0 == key) {
                return RouteColliders(guard);
            }
        }
        let built = self.build_route_colliders();
        *self
            .route_cache
            .0
            .write()
            .unwrap_or_else(|e| e.into_inner()) = Some((key, built));
        RouteColliders(read())
    }

    /// The name of a route collider a collision pair points at.
    pub fn route_collider_name(&self, k: usize) -> String {
        self.route_colliders()
            .get(k)
            .map(|rc| rc.name.clone())
            .unwrap_or_else(|| format!("route#{k}"))
    }
}

/// A face of a body's bounds, `+x` … `-z`, as (axis, side).
fn face_axis(face: &str) -> Option<(usize, f64)> {
    match face.trim() {
        "+x" | "x" => Some((0, 1.0)),
        "-x" => Some((0, -1.0)),
        "+y" | "y" => Some((1, 1.0)),
        "-y" => Some((1, -1.0)),
        "+z" | "z" => Some((2, 1.0)),
        "-z" => Some((2, -1.0)),
        _ => None,
    }
}

/// The rope tracks a replay of one span is named by (`bt.routing.animate`
/// writes them): the span's own, `route:<route>#<k>` with `k` its index in
/// the route's spans, and — for a stretch between two holders of one
/// carrier — the hose's, `carrier:<c>#<i>`, which every line in it reads.
/// The studio and the USD export draw a span the cycle replays as its rope,
/// not as its design line.
pub fn span_rope_names(route: &str, spans: &[Span], index: usize) -> Vec<String> {
    let start = stretch_start(spans, index);
    let span = &spans[start];
    let mut names = vec![format!("route:{route}#{start}")];
    if let Some((carrier, i)) = carrier_span_of(span) {
        names.push(format!("carrier:{carrier}#{i}"));
    }
    names
}

/// The first span of the stretch `index` belongs to: the spans joined
/// through sliding holders, which share their material and are replayed
/// as one line, named after the first of them.
pub fn stretch_start(spans: &[Span], index: usize) -> usize {
    let mut start = index.min(spans.len().saturating_sub(1));
    while start > 0 && spans[start].slides {
        start -= 1;
    }
    start
}

/// The last span of the stretch `index` belongs to.
pub fn stretch_end(spans: &[Span], index: usize) -> usize {
    let mut end = index;
    while end + 1 < spans.len() && spans[end + 1].slides {
        end += 1;
    }
    end
}

/// The carrier and holder index of a span that runs inside a carrier,
/// from holder `i` to holder `i + 1`.
fn carrier_span_of(span: &Span) -> Option<(String, usize)> {
    let (c0, i) = Support::holder_of(&span.from)?;
    let (c1, j) = Support::holder_of(&span.to)?;
    (c0 == c1 && j == i + 1).then(|| (c0.to_string(), i))
}

/// The links rigidly joined to `link` (through fixed joints, both ways),
/// `link` included: a connector sits on a body, not on one of the links a
/// description splits it into (`base_link` / `base_link_inertia`).
/// Whether a link moves with the joints: one below a joint that turns or
/// slides — not the root, nor a link fixed to it (a base casting, a
/// flange plate on the base).
fn link_moves(model: &botrail_model::RobotModel, link: usize) -> bool {
    !rigid_group(model, model.root_link).contains(&link)
}

fn rigid_group(model: &botrail_model::RobotModel, link: usize) -> Vec<usize> {
    let mut group = vec![link];
    let mut i = 0;
    while i < group.len() {
        let current = group[i];
        for joint in &model.joints {
            if joint.joint_type != botrail_model::JointType::Fixed {
                continue;
            }
            for other in [joint.parent_link, joint.child_link] {
                if (joint.parent_link == current || joint.child_link == current)
                    && !group.contains(&other)
                {
                    group.push(other);
                }
            }
        }
        i += 1;
    }
    group
}

/// The point of a polyline nearest `q`, with its arc length along it.
fn closest_on_polyline(path: &[Point3<f64>], q: &Point3<f64>) -> (Point3<f64>, f64) {
    let mut best = (path[0], 0.0, f64::INFINITY);
    let mut s0 = 0.0;
    for w in path.windows(2) {
        let d = w[1] - w[0];
        let len = d.norm();
        let t = if len > EPS {
            ((q - w[0]).dot(&d) / (len * len)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let p = w[0] + d * t;
        let dist = (q - p).norm();
        if dist < best.2 {
            best = (p, s0 + t * len, dist);
        }
        s0 += len;
    }
    (best.0, best.1)
}

/// The stretch of a polyline between two arc lengths, in travel order
/// (reversed when `to_s < from_s`), both cut points included.
fn polyline_between(path: &[Point3<f64>], from_s: f64, to_s: f64) -> Vec<Point3<f64>> {
    let at = |target: f64| -> Point3<f64> {
        let mut s0 = 0.0;
        for w in path.windows(2) {
            let len = (w[1] - w[0]).norm();
            if target <= s0 + len + EPS {
                let t = if len > EPS {
                    ((target - s0) / len).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                return w[0] + (w[1] - w[0]) * t;
            }
            s0 += len;
        }
        path[path.len() - 1]
    };
    let (lo, hi) = if from_s <= to_s {
        (from_s, to_s)
    } else {
        (to_s, from_s)
    };
    let mut out = vec![at(lo)];
    let mut s0 = 0.0;
    for w in path.windows(2) {
        s0 += (w[1] - w[0]).norm();
        if s0 > lo + EPS && s0 < hi - EPS {
            out.push(w[1]);
        }
    }
    out.push(at(hi));
    if from_s > to_s {
        out.reverse();
    }
    out
}

/// Where two routes' fixed spans touch, if anywhere (capsule against
/// capsule at the routes' radii).
fn fixed_spans_touch(a: &RouteGeometry, b: &RouteGeometry) -> Option<[f64; 3]> {
    let ra = a.od_m.map(|d| d / 2.0).unwrap_or(0.0);
    let rb = b.od_m.map(|d| d / 2.0).unwrap_or(0.0);
    let segments = |g: &RouteGeometry| -> Vec<(Point3<f64>, Point3<f64>)> {
        g.spans
            .iter()
            .filter(|s| s.kind == SpanKind::Fixed)
            .flat_map(|s| {
                let pts = g.span_points(s);
                pts.windows(2)
                    .map(|w| (Point3::from(w[0]), Point3::from(w[1])))
                    .collect::<Vec<_>>()
            })
            .collect()
    };
    let sa = segments(a);
    let sb = segments(b);
    for (p1, q1) in &sa {
        for (p2, q2) in &sb {
            if segment_distance(*p1, *q1, *p2, *q2) <= ra + rb + 1e-9 {
                return Some(to_array(nalgebra::center(p1, q1)));
            }
        }
    }
    None
}

/// Closest distance between two segments.
fn segment_distance(p1: Point3<f64>, q1: Point3<f64>, p2: Point3<f64>, q2: Point3<f64>) -> f64 {
    // Ericson, Real-Time Collision Detection, 5.1.9.
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);
    let (mut s, mut t);
    if a <= EPS && e <= EPS {
        return r.norm();
    }
    if a <= EPS {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(&r);
        if e <= EPS {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;
            s = if denom != 0.0 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }
    ((p1 + d1 * s) - (p2 + d2 * t)).norm()
}

// ---------------------------------------------------------------------- BOM

fn category(kind: LineKind, medium: Option<Medium>) -> String {
    match kind {
        LineKind::Cable => match medium {
            Some(m) => format!("cable.{}", medium_name(m)),
            None => "cable".to_string(),
        },
        LineKind::Tube => "pneumatic.tube".to_string(),
        LineKind::Hose => "pneumatic.hose".to_string(),
    }
}

/// The BOM lines routing derives: one per route (the line, by its spec,
/// with its cut and purchase length — identical specs merge and their
/// lengths add) and one per clamp article. Ducts are generated structures
/// with parts of their own.
pub(crate) fn bom_rows(scene: &Scene) -> Vec<BomRow> {
    let plan = scene.routing_plan();
    let report = scene.routing_report();
    let mut rows = Vec::new();
    for (route, geometry) in plan.routes.iter().zip(&report.geometries) {
        let spec = plan.spec(&route.spec);
        let mut attributes = BTreeMap::new();
        attributes.insert("spec".to_string(), PartAttr::Text(route.spec.clone()));
        if let Some(od) = spec.and_then(|s| s.od_m) {
            attributes.insert(
                "od_mm".to_string(),
                PartAttr::Number((od * 1e4).round() / 10.0),
            );
        }
        if let Some(g) = geometry {
            attributes.insert(
                "length_m".to_string(),
                PartAttr::Number(round_m(g.cut_length_m)),
            );
            if let Some(p) = g.purchase_length_m {
                attributes.insert(
                    "purchase_length_m".to_string(),
                    PartAttr::Number(round_m(p)),
                );
            }
            if let Some(mass) = spec.and_then(|s| s.mass_kg_m) {
                attributes.insert(
                    "mass_kg".to_string(),
                    PartAttr::Number(round_m(
                        mass * g.purchase_length_m.unwrap_or(g.cut_length_m),
                    )),
                );
            }
        }
        rows.push(BomRow {
            category: category(
                spec.map(|s| s.kind).unwrap_or_default(),
                geometry.as_ref().and_then(|g| g.medium),
            ),
            names: vec![route.name.clone()],
            manufacturer: spec.and_then(|s| s.manufacturer.clone()),
            model: spec.and_then(|s| s.model.clone()),
            catalog: spec
                .and_then(|s| s.catalog.as_deref())
                .map(CatalogRef::parse),
            qty: 1,
            description: spec
                .map(|s| s.name.clone())
                .or_else(|| Some(format!("spec `{}` is not declared", route.spec))),
            attributes,
            order: None,
            targets: vec![BomTarget {
                kind: PartTargetKind::Route,
                name: route.name.clone(),
                derived: true,
            }],
        });
    }
    // Clamps: one line per article; the unidentified ones one line together.
    let mut groups: ClampGroups = BTreeMap::new();
    for support in &plan.supports {
        if let Support::Clamp { name, .. } = support {
            let (manufacturer, model, catalog) = support.identity();
            groups
                .entry((
                    manufacturer.map(str::to_string),
                    model.map(str::to_string),
                    catalog.map(str::to_string),
                ))
                .or_default()
                .push((name.clone(), name.clone()));
        }
    }
    // The clamps a dressed run generated: the route's article, counted
    // under the route.
    for (route, geometry) in plan.routes.iter().zip(&report.geometries) {
        let Some(g) = geometry else { continue };
        if g.auto_clamps.is_empty() {
            continue;
        }
        let article = route.clamp.clone().unwrap_or_default();
        groups
            .entry((article.manufacturer, article.model, article.catalog))
            .or_default()
            .extend(
                g.auto_clamps
                    .iter()
                    .map(|c| (c.clone(), route.name.clone())),
            );
    }
    for support in &plan.supports {
        let Support::Carrier {
            name,
            holders,
            inner_m,
            od_m,
            bend_r_moving_m,
            torsion_deg_m,
            stroke_m,
            ..
        } = support
        else {
            continue;
        };
        // The hose: as long as the longest route dressed in it runs between
        // its first and last holder; the line between the holders is that
        // run plus the slack the route brings into the hose's spans — what
        // a kit's hose length is asked to cover (`free_length_m`).
        let first = Support::holder_name(name, 0);
        let last = Support::holder_name(name, holders.len().saturating_sub(1));
        let mut length: Option<f64> = None;
        let mut free: Option<f64> = None;
        for (route, geometry) in plan.routes.iter().zip(&report.geometries) {
            let Some(g) = geometry else { continue };
            if !g.carriers.iter().any(|c| c == name) {
                continue;
            }
            let (Some(a0), Some(a1)) = (
                g.anchors.iter().find(|a| a.name == first),
                g.anchors.iter().find(|a| a.name == last),
            ) else {
                continue;
            };
            let run = (a1.s_m - a0.s_m).abs();
            length = Some(length.map_or(run, |l: f64| l.max(run)));
            let moving = g
                .spans
                .iter()
                .filter(|s| s.kind == SpanKind::Moving)
                .count();
            let mut moving_seen = 0usize;
            let mut slack = 0.0;
            for span in &g.spans {
                if span.kind == SpanKind::Moving {
                    if carrier_span_of(span).is_some_and(|(c, _)| c == *name) {
                        slack += route.slack_of_moving(moving_seen, moving);
                    }
                    moving_seen += 1;
                }
            }
            free = Some(free.map_or(run + slack, |f: f64| f.max(run + slack)));
        }
        let (manufacturer, model, catalog) = support.identity();
        let mut attributes = BTreeMap::new();
        attributes.insert(
            "holders".to_string(),
            PartAttr::Number(holders.len() as f64),
        );
        for (key, value) in [
            ("inner_mm", inner_m.map(|v| (v * 1e4).round() / 10.0)),
            ("od_mm", od_m.map(|v| (v * 1e4).round() / 10.0)),
            (
                "bend_r_moving_mm",
                bend_r_moving_m.map(|v| (v * 1e4).round() / 10.0),
            ),
            ("torsion_deg_m", *torsion_deg_m),
            ("stroke_m", *stroke_m),
            ("free_length_m", free.map(round_m)),
        ] {
            if let Some(v) = value {
                attributes.insert(key.to_string(), PartAttr::Number(v));
            }
        }
        if let Some(l) = length {
            attributes.insert("length_m".to_string(), PartAttr::Number(round_m(l)));
        }
        rows.push(BomRow {
            category: "cable.carrier".to_string(),
            names: vec![name.clone()],
            manufacturer: manufacturer.map(str::to_string),
            model: model.map(str::to_string),
            catalog: catalog.map(CatalogRef::parse),
            qty: 1,
            description: Some("protective hose / carrier with its holders".to_string()),
            attributes,
            order: None,
            targets: vec![BomTarget {
                kind: PartTargetKind::Route,
                name: name.clone(),
                derived: true,
            }],
        });
    }
    for ((manufacturer, model, catalog), members) in groups {
        let mut targets: Vec<BomTarget> = Vec::new();
        for (_, target) in &members {
            if !targets.iter().any(|t| &t.name == target) {
                targets.push(BomTarget {
                    kind: PartTargetKind::Route,
                    name: target.clone(),
                    derived: true,
                });
            }
        }
        let names: Vec<String> = members.into_iter().map(|(n, _)| n).collect();
        rows.push(BomRow {
            category: "cable.clamp".to_string(),
            qty: names.len() as u32,
            targets,
            names,
            manufacturer,
            model,
            catalog: catalog.as_deref().map(CatalogRef::parse),
            description: Some("cable clamp".to_string()),
            attributes: BTreeMap::new(),
            order: None,
        });
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::connections::{ConnectionPlan, Port, Role};
    use botrail_model::Geometry;

    fn boxed(scene: &mut Scene, name: &str, at: [f64; 3]) {
        scene
            .add_obstacle(
                name,
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                Isometry3::translation(at[0], at[1], at[2]),
            )
            .unwrap();
    }

    fn port(name: &str, target: &str, role: Role, at: Attach, exit: [f64; 3]) -> Port {
        Port {
            name: name.into(),
            target: target.into(),
            target_kind: PartTargetKind::Obstacle,
            medium: Medium::Signal,
            role,
            required: true,
            specs: Default::default(),
            io: None,
            terminal: None,
            reference: None,
            geometry: Some(PortGeometry {
                at,
                exit,
                straight_m: 0.05,
            }),
        }
    }

    fn cell() -> Scene {
        let mut scene = Scene::empty();
        boxed(&mut scene, "box_a", [0.0, 0.0, 0.1]);
        boxed(&mut scene, "box_b", [2.0, 1.0, 0.1]);
        scene
            .set_connection_plan(ConnectionPlan {
                ports: vec![
                    port(
                        "a.out",
                        "box_a",
                        Role::Output,
                        Attach::Obstacle {
                            name: "box_a".into(),
                            offset_m: [0.1, 0.0, 0.0],
                        },
                        [1.0, 0.0, 0.0],
                    ),
                    port(
                        "b.in",
                        "box_b",
                        Role::Input,
                        Attach::Obstacle {
                            name: "box_b".into(),
                            offset_m: [-0.1, 0.0, 0.0],
                        },
                        [-1.0, 0.0, 0.0],
                    ),
                ],
                links: vec![Connection {
                    name: "w1".into(),
                    source: "a.out".into(),
                    target: "b.in".into(),
                    cable: None,
                    reference: None,
                }],
            })
            .unwrap();
        scene
            .set_routing_plan(RoutingPlan {
                specs: vec![CableSpec {
                    name: "sig".into(),
                    od_m: Some(0.008),
                    bend_r_fixed_m: Some(0.05),
                    sold_lengths_m: vec![2.0, 5.0],
                    end_allowance_m: 0.1,
                    ..Default::default()
                }],
                supports: Vec::new(),
                routes: vec![Route {
                    name: "r1".into(),
                    carries: vec!["w1".into()],
                    spec: "sig".into(),
                    via: Vec::new(),
                    orthogonal: true,
                    slack_m: 0.0,
                    slacks_m: Vec::new(),
                    collide: true,
                    color: None,
                    clamp_pitch_m: None,
                    clamp: None,
                }],
            })
            .unwrap();
        scene
    }

    #[test]
    fn a_span_is_replayed_under_its_own_name_or_its_hose() {
        let span = |from: &str, to: &str| Span {
            kind: SpanKind::Moving,
            from: from.into(),
            to: to.into(),
            from_s: 0.0,
            to_s: 1.0,
            slides: false,
        };
        let spans = vec![
            span("valve/out", "dress/h0"),
            span("dress/h0", "dress/h1"),
            span("dress/h1", "pack/h2"),
            span("dress/h0", "dress/h2"),
        ];
        assert_eq!(span_rope_names("air", &spans, 0), vec!["route:air#0"]);
        assert_eq!(
            span_rope_names("air", &spans, 1),
            vec!["route:air#1", "carrier:dress#0"]
        );
        // Two holders of different carriers, or not neighbours: no hose.
        assert_eq!(span_rope_names("air", &spans, 2), vec!["route:air#2"]);
        assert_eq!(span_rope_names("air", &spans, 3), vec!["route:air#3"]);
        // A span the line slides into shares the one before it.
        let mut sliding = spans.clone();
        sliding[2].slides = true;
        assert_eq!(stretch_start(&sliding, 2), 1);
        assert_eq!(stretch_end(&sliding, 1), 2);
        assert_eq!(
            span_rope_names("air", &sliding, 2),
            vec!["route:air#1", "carrier:dress#0"]
        );
    }

    #[test]
    fn a_rolling_holder_takes_the_turn_about_the_line_out_of_the_twist() {
        use nalgebra::UnitQuaternion;
        let roll = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.2);
        let bend = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.4);
        assert!((swing_angle(&roll, &[]) - 1.2).abs() < 1e-9);
        assert!(
            swing_angle(&roll, &[Vector3::z()]).abs() < 1e-9,
            "a pure roll about the free axis is no twist"
        );
        assert!(
            (swing_angle(&bend, &[Vector3::z()]) - 0.4).abs() < 1e-9,
            "a bend is kept"
        );
        let both = bend * roll;
        let left = swing_angle(&both, &[Vector3::z()]);
        assert!(left < both.angle() && (left - 0.4).abs() < 1e-9, "{left}");
        // Twist is read the same way for either sign of the axis.
        assert!((swing_angle(&both, &[-Vector3::z()]) - left).abs() < 1e-9);
    }

    #[test]
    fn holder_kinds_are_the_plan_s_words_and_a_span_slides_after_a_sliding_holder() {
        let kinds: Vec<HolderKind> =
            serde_json::from_str(r#"["fixed", "rotating", "sliding", "strap"]"#).unwrap();
        assert_eq!(
            kinds,
            vec![
                HolderKind::Fixed,
                HolderKind::Rotating,
                HolderKind::Sliding,
                HolderKind::Strap
            ]
        );
        assert_eq!(HolderKind::default(), HolderKind::Fixed);
        assert!(
            !HolderKind::Fixed.rolls() && HolderKind::Rotating.rolls() && HolderKind::Strap.rolls()
        );
        assert!(HolderKind::Sliding.slides() && !HolderKind::Rotating.slides());
        assert!(HolderKind::Rotating.holds_direction() && !HolderKind::Strap.holds_direction());
        let clamp: Support = serde_json::from_str(
            r#"{"kind": "clamp", "name": "c", "at": {"kind": "obstacle", "name": "box", "offset_m": [0, 0, 0]}, "hold": "sliding", "direction": [0, 0, 1]}"#,
        )
        .unwrap();
        match clamp {
            Support::Clamp {
                hold, direction, ..
            } => {
                assert_eq!(hold, HolderKind::Sliding);
                assert_eq!(direction, Some([0.0, 0.0, 1.0]));
            }
            _ => panic!("a clamp"),
        }
        let carrier: Support = serde_json::from_str(
            r#"{"kind": "carrier", "name": "d", "holders": [{"at": {"kind": "obstacle", "name": "box", "offset_m": [0, 0, 0]}}, {"at": {"kind": "obstacle", "name": "box", "offset_m": [0, 0, 0.1]}, "kind": "rotating"}]}"#,
        )
        .unwrap();
        match carrier {
            Support::Carrier { holders, .. } => {
                assert_eq!(holders[0].kind, HolderKind::Fixed);
                assert_eq!(holders[1].kind, HolderKind::Rotating);
            }
            _ => panic!("a carrier"),
        }
    }

    #[test]
    fn a_run_along_a_column_makes_clamps_on_its_near_face() {
        // A column between the boxes: the route hugs its -y face (nearest
        // both ways), 3 cm off the surface, clamps every 0.25 m, the run
        // parallel to the column's edge.
        let mut scene = cell();
        scene
            .add_obstacle(
                "column",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 1.0),
                },
                Isometry3::translation(1.0, 0.5, 0.5),
            )
            .unwrap();
        let mut plan = scene.routing_plan().clone();
        plan.routes[0].orthogonal = false;
        plan.routes[0].clamp_pitch_m = Some(0.25);
        plan.routes[0].clamp = Some(ClampArticle {
            model: Some("P-clip 8".into()),
            standoff_m: Some(0.026),
            ..Default::default()
        });
        plan.routes[0].via = vec![
            Via::Point {
                position: [1.0, 0.0, 0.1],
            },
            Via::Along {
                on: Body::Obstacle {
                    name: "column".into(),
                },
                pitch_m: None,
                face: None,
                offset_m: None,
                inset_m: None,
            },
            Via::Point {
                position: [1.0, 0.0, 0.9],
            },
        ];
        scene.set_routing_plan(plan).unwrap();
        let g = scene.route_geometry("r1").unwrap();
        // 0.1 → 0.9 (within the face's inset, 0.05 .. 0.95): 0.8 m of run,
        // four intervals of at most a pitch, five clamps.
        assert_eq!(
            g.auto_clamps,
            vec!["r1/c0", "r1/c1", "r1/c2", "r1/c3", "r1/c4"]
        );
        let clamps: Vec<&RouteAnchor> = g.anchors.iter().filter(|a| a.auto).collect();
        assert_eq!(clamps.len(), 5);
        for c in &clamps {
            assert!(c.kind == AnchorKind::Clamp && !c.moving);
            // -y face: 0.5 - 0.1 - (0.004 + 0.026) = 0.37.
            assert!((c.position[1] - 0.37).abs() < 1e-9, "{:?}", c.position);
            assert!((c.position[0] - 1.0).abs() < 1e-9);
            assert!(matches!(&c.attach, Some(Attach::Obstacle { name, .. }) if name == "column"));
        }
        assert!(
            (clamps[0].position[2] - 0.1).abs() < 1e-9
                && (clamps[4].position[2] - 0.9).abs() < 1e-9
        );
        assert!(g.spans.iter().all(|s| s.kind == SpanKind::Fixed));
        assert_eq!(g.spans.len(), 6);
        // The clamps are a BOM line of their article, counted under the route.
        let report = scene.routing_report();
        assert!(
            !report
                .findings
                .iter()
                .any(|f| f.code == "route_clash" || f.code == "route_clearance"),
            "{:?}",
            report.findings
        );
        let rows = bom_rows(&scene);
        let clamp_row = rows
            .iter()
            .find(|r| r.category == "cable.clamp")
            .expect("clamp row");
        assert_eq!(clamp_row.qty, 5);
        assert_eq!(clamp_row.model.as_deref(), Some("P-clip 8"));
        assert_eq!(clamp_row.targets.len(), 1);
        assert_eq!(clamp_row.targets[0].name, "r1");
        assert_eq!(report.routes[0].auto_clamps, 5);
    }

    #[test]
    fn abutting_fillets_leave_no_zero_length_segment() {
        // Two right-angle corners 0.1 m apart with r = 0.05: each fillet's
        // tangent is 0.05, so the arcs meet at one point — once, not twice.
        let mut scene = cell();
        let mut plan = scene.routing_plan().clone();
        plan.routes[0].orthogonal = false;
        plan.routes[0].via = vec![
            Via::Point {
                position: [1.0, 0.0, 0.1],
            },
            Via::Point {
                position: [1.0, 0.1, 0.1],
            },
            Via::Point {
                position: [1.1, 0.1, 0.1],
            },
        ];
        scene.set_routing_plan(plan).unwrap();
        let g = scene.route_geometry("r1").unwrap();
        assert_eq!(
            g.bends.iter().filter(|b| b.ok == Some(true)).count(),
            g.bends.len()
        );
        assert!(g.bends.len() >= 2, "{:?}", g.bends);
        for w in g.centerline.windows(2) {
            let d = Point3::from(w[0]) - Point3::from(w[1]);
            assert!(d.norm() > 1e-9, "zero-length segment at {:?}", w[0]);
        }
        assert!(g.s.windows(2).all(|w| w[1] > w[0]));
    }

    #[test]
    fn orthogonal_route_follows_the_boxes() {
        let scene = cell();
        let g = scene.route_geometry("r1").unwrap();
        // 0.1 + 0.05 stub, then x to 1.85, y to 1.0, then the stub into b.
        // Two right-angle fillets, tessellated at 5°, each a hair short of the exact arc.
        assert!(
            (g.length_m - (2.8 - fillet_saving(2, 0.05))).abs() < 1e-3,
            "{}",
            g.length_m
        );
        assert_eq!(g.spans.len(), 1);
        assert_eq!(g.spans[0].kind, SpanKind::Fixed);
        assert_eq!(g.bends.len(), 2);
        assert!(g.bends.iter().all(|b| b.ok == Some(true)));
        assert_eq!(g.purchase_length_m, Some(5.0));
        assert!(g.cut_length_m > 2.0);
        let report = scene.routing_report();
        assert!(report.errors().next().is_none(), "{:?}", report.findings);
        assert_eq!(report.routes[0].status, "ok");
    }

    /// Two right-angle fillets of radius r shorten a polyline by
    /// 2r − πr/2 each.
    fn fillet_saving(count: usize, r: f64) -> f64 {
        count as f64 * (2.0 * r - PI * r / 2.0)
    }

    #[test]
    fn moving_the_box_moves_the_length_and_the_sold_length() {
        let mut scene = cell();
        let before = scene.route_geometry("r1").unwrap();
        scene
            .set_obstacle_pose("box_b", Isometry3::translation(5.0, 1.0, 0.1))
            .unwrap();
        let after = scene.route_geometry("r1").unwrap();
        assert!((after.length_m - before.length_m - 3.0).abs() < 1e-6);
        assert_eq!(after.purchase_length_m, None);
        let report = scene.routing_report();
        assert!(report
            .findings
            .iter()
            .any(|f| f.code == "route_length" && f.severity == "error"));
    }

    #[test]
    fn a_tight_corner_is_a_bend_finding_with_a_place() {
        let mut scene = cell();
        let mut plan = scene.routing_plan().clone();
        plan.specs[0].bend_r_fixed_m = Some(0.6);
        scene.set_routing_plan(plan).unwrap();
        let report = scene.routing_report();
        let bend = report
            .findings
            .iter()
            .find(|f| f.code == "route_bend")
            .expect("a bend finding");
        assert!(bend.position.is_some() && bend.s_m.is_some());
        assert_eq!(report.routes[0].status, "error");
    }

    #[test]
    fn a_box_in_the_way_is_a_clash_and_a_duct_is_not() {
        let mut scene = cell();
        boxed(&mut scene, "wall", [1.0, 0.0, 0.1]);
        let report = scene.routing_report();
        assert!(report
            .findings
            .iter()
            .any(|f| f.code == "route_clash" && f.message.contains("wall")));
        // The same box as a duct the route runs through is expected contact.
        scene.remove_obstacle("wall").unwrap();
        boxed(&mut scene, "duct/run0", [1.0, 0.0, 0.1]);
        let mut plan = scene.routing_plan().clone();
        plan.supports.push(Support::Duct {
            name: "duct".into(),
            path: vec![[0.5, 0.0, 0.1], [1.5, 0.0, 0.1]],
            inner_m: [0.1, 0.05],
            manufacturer: None,
            model: None,
            catalog: None,
        });
        plan.routes[0].via = vec![Via::Support {
            name: "duct".into(),
        }];
        scene.set_routing_plan(plan).unwrap();
        let report = scene.routing_report();
        assert!(
            !report.findings.iter().any(|f| f.code == "route_clash"),
            "{:?}",
            report.findings
        );
        assert_eq!(report.routes[0].ducts, vec!["duct".to_string()]);
    }

    #[test]
    fn bom_rows_carry_the_length_and_the_plan_round_trips() {
        let scene = cell();
        let rows = bom_rows(&scene);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].category, "cable.signal");
        assert!(
            rows[0]
                .attributes
                .get("length_m")
                .and_then(PartAttr::as_number)
                .unwrap()
                > 2.0
        );
        let json = serde_json::to_string(scene.routing_plan()).unwrap();
        let back: RoutingPlan = serde_json::from_str(&json).unwrap();
        assert_eq!(&back, scene.routing_plan());
    }

    #[test]
    fn unrouted_connections_are_the_to_do() {
        let mut scene = cell();
        let mut plan = scene.routing_plan().clone();
        plan.routes.clear();
        scene.set_routing_plan(plan).unwrap();
        let report = scene.routing_report();
        assert!(report
            .findings
            .iter()
            .any(|f| f.code == "route_unrouted" && f.route == "w1"));
    }
}
