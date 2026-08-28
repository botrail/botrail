//! The plan-view layout sheet — the cell seen from above, as the drawing
//! a proposal or a factory layout wants: every piece of equipment as its
//! footprint, robots as base marks with their reach, conveyor zones and
//! sensor beams, mount frames, an overall dimension, and labels on the
//! things that have names worth printing.
//!
//! Like the BOM, the sheet is *derived*: nothing here is authored, it is
//! the scene projected onto the floor. It renders to SVG (self-contained,
//! no dependency) and to a minimal R12 DXF (LINE / POLYLINE / CIRCLE /
//! TEXT on named layers), so it opens in a browser and in any 2D CAD.
//! Both come from one [`LayoutSheet`], so the SVG a reviewer looks at and
//! the DXF the layout engineer imports say the same thing.
//!
//! What it deliberately is not: a drawing with tolerances, a title block
//! standard, or a 3D-to-2D projection of every prim. Footprints are
//! convex hulls of primitives and bounding boxes of meshes; the ground
//! (floor slabs, painted markings — anything under `ground_z`) is drawn
//! faint and left out of the extents.

use nalgebra::{Isometry3, Point3};
use serde::Serialize;

use botrail_model::{Geometry, RobotSource};

use crate::part::PartTargetKind;
use crate::seq::{DeviceKind, SensorKind};
use crate::Scene;

/// Where a sheet item is drawn from — its layer, which the renderers map
/// to a style (SVG class) or a DXF layer name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LayoutLayer {
    /// Floor slabs and painted markings: faint, not part of the extents.
    Ground,
    /// Obstacles — machines, tables, fences, workpieces.
    Equipment,
    /// Robot base marks.
    Robot,
    /// Robot reach envelopes (catalog `reach_mm`), dashed.
    Reach,
    /// Device zones and paths: conveyor and sink zones, axis travel,
    /// vehicle routes.
    Device,
    /// Sensor zones and beams.
    Sensor,
    /// Named frames (mount points, teach references).
    Frame,
    /// Text labels.
    Label,
    /// Overall dimensions.
    Dimension,
    /// The metre grid.
    Grid,
}

impl LayoutLayer {
    fn dxf_name(self) -> &'static str {
        match self {
            LayoutLayer::Ground => "GROUND",
            LayoutLayer::Equipment => "EQUIPMENT",
            LayoutLayer::Robot => "ROBOT",
            LayoutLayer::Reach => "REACH",
            LayoutLayer::Device => "DEVICE",
            LayoutLayer::Sensor => "SENSOR",
            LayoutLayer::Frame => "FRAME",
            LayoutLayer::Label => "LABEL",
            LayoutLayer::Dimension => "DIM",
            LayoutLayer::Grid => "GRID",
        }
    }

    /// AutoCAD colour index for the DXF layer table.
    fn dxf_color(self) -> u32 {
        match self {
            LayoutLayer::Ground => 8,
            LayoutLayer::Equipment => 7,
            LayoutLayer::Robot => 2,
            LayoutLayer::Reach => 2,
            LayoutLayer::Device => 5,
            LayoutLayer::Sensor => 1,
            LayoutLayer::Frame => 3,
            LayoutLayer::Label => 7,
            LayoutLayer::Dimension => 7,
            LayoutLayer::Grid => 9,
        }
    }

    fn svg_class(self) -> &'static str {
        match self {
            LayoutLayer::Ground => "ground",
            LayoutLayer::Equipment => "equipment",
            LayoutLayer::Robot => "robot",
            LayoutLayer::Reach => "reach",
            LayoutLayer::Device => "device",
            LayoutLayer::Sensor => "sensor",
            LayoutLayer::Frame => "frame",
            LayoutLayer::Label => "label",
            LayoutLayer::Dimension => "dim",
            LayoutLayer::Grid => "grid",
        }
    }

    const ALL: [LayoutLayer; 10] = [
        LayoutLayer::Ground,
        LayoutLayer::Equipment,
        LayoutLayer::Robot,
        LayoutLayer::Reach,
        LayoutLayer::Device,
        LayoutLayer::Sensor,
        LayoutLayer::Frame,
        LayoutLayer::Label,
        LayoutLayer::Dimension,
        LayoutLayer::Grid,
    ];
}

/// One drawn thing, in world metres (x right, y up — the plan view of a
/// Z-up world).
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum LayoutShape {
    /// A closed outline.
    Polygon {
        points: Vec<[f64; 2]>,
    },
    Circle {
        center: [f64; 2],
        radius: f64,
    },
    Line {
        from: [f64; 2],
        to: [f64; 2],
    },
    /// An open polyline (a vehicle route).
    Polyline {
        points: Vec<[f64; 2]>,
    },
    /// A text anchored at `at` (centred), `size` in metres.
    Text {
        at: [f64; 2],
        text: String,
        size: f64,
    },
    /// A cross mark (a frame), `size` in metres.
    Cross {
        at: [f64; 2],
        size: f64,
    },
    /// An arrow from `from` to `to` (a direction of travel).
    Arrow {
        from: [f64; 2],
        to: [f64; 2],
    },
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LayoutItem {
    pub layer: LayoutLayer,
    /// The scene name the item stands for (an obstacle, a robot, a device,
    /// a sensor, a frame) — empty for decoration (grid, dimensions).
    pub name: String,
    pub shape: LayoutShape,
    pub dashed: bool,
}

/// The overall plan-view extent of the equipment (ground excluded), in
/// metres.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Footprint {
    pub min: [f64; 2],
    pub max: [f64; 2],
    /// Height of the tallest non-ground item above z = 0.
    pub height: f64,
}

impl Footprint {
    pub fn width(&self) -> f64 {
        (self.max[0] - self.min[0]).max(0.0)
    }
    pub fn depth(&self) -> f64 {
        (self.max[1] - self.min[1]).max(0.0)
    }
    pub fn area(&self) -> f64 {
        self.width() * self.depth()
    }

    fn include(&mut self, p: [f64; 2]) {
        self.min[0] = self.min[0].min(p[0]);
        self.min[1] = self.min[1].min(p[1]);
        self.max[0] = self.max[0].max(p[0]);
        self.max[1] = self.max[1].max(p[1]);
    }

    fn empty() -> Footprint {
        Footprint {
            min: [f64::INFINITY; 2],
            max: [f64::NEG_INFINITY; 2],
            height: 0.0,
        }
    }

    fn is_empty(&self) -> bool {
        self.min[0] > self.max[0] || self.min[1] > self.max[1]
    }
}

/// What to draw and how to read the world.
#[derive(Debug, Clone, PartialEq)]
pub struct LayoutOptions {
    /// Obstacles whose top is at or below this height are ground (floor
    /// slabs, painted markings): drawn faint, excluded from the extents.
    pub ground_z: f64,
    /// Draw named frames.
    pub frames: bool,
    /// Draw labels (part / group / equipment names).
    pub labels: bool,
    /// Draw robot reach circles where the catalog knows the reach.
    pub reach: bool,
    /// Grid pitch in metres; `None` for no grid.
    pub grid: Option<f64>,
    /// Sheet title (defaults to the first robot's name when empty).
    pub title: String,
}

impl Default for LayoutOptions {
    fn default() -> Self {
        LayoutOptions {
            ground_z: 0.02,
            frames: true,
            labels: true,
            reach: true,
            grid: Some(1.0),
            title: String::new(),
        }
    }
}

/// The plan-view sheet: extents plus the items to draw. Built by
/// [`Scene::layout`], rendered by [`LayoutSheet::to_svg`] and
/// [`LayoutSheet::to_dxf`].
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LayoutSheet {
    pub title: String,
    pub footprint: Footprint,
    pub items: Vec<LayoutItem>,
}

// ------------------------------------------------------------- geometry

/// Convex hull of a point set (Andrew's monotone chain), counter-clockwise,
/// without the closing duplicate. Degenerate inputs come back as-is.
pub fn convex_hull(mut points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    points.retain(|p| p[0].is_finite() && p[1].is_finite());
    points.sort_by(|a, b| {
        a[0].partial_cmp(&b[0])
            .unwrap()
            .then(a[1].partial_cmp(&b[1]).unwrap())
    });
    points.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-12 && (a[1] - b[1]).abs() < 1e-12);
    if points.len() < 3 {
        return points;
    }
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    let mut lower: Vec<[f64; 2]> = Vec::new();
    for &p in &points {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<[f64; 2]> = Vec::new();
    for &p in points.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn xy(p: Point3<f64>) -> [f64; 2] {
    [p.x, p.y]
}

/// The plan-view outline of a primitive at `pose`, plus its z range.
/// Meshes are handled by the caller from their collider bounds.
fn primitive_footprint(
    geometry: &Geometry,
    pose: &Isometry3<f64>,
) -> Option<(LayoutShape, (f64, f64))> {
    match geometry {
        Geometry::Box { size } => {
            let h = size / 2.0;
            let mut pts = Vec::with_capacity(8);
            let mut zmin = f64::INFINITY;
            let mut zmax = f64::NEG_INFINITY;
            for sx in [-1.0, 1.0] {
                for sy in [-1.0, 1.0] {
                    for sz in [-1.0, 1.0] {
                        let p = pose * Point3::new(sx * h.x, sy * h.y, sz * h.z);
                        zmin = zmin.min(p.z);
                        zmax = zmax.max(p.z);
                        pts.push(xy(p));
                    }
                }
            }
            Some((
                LayoutShape::Polygon {
                    points: convex_hull(pts),
                },
                (zmin, zmax),
            ))
        }
        Geometry::Sphere { radius } => {
            let c = pose.translation.vector;
            Some((
                LayoutShape::Circle {
                    center: [c.x, c.y],
                    radius: *radius,
                },
                (c.z - radius, c.z + radius),
            ))
        }
        Geometry::Cylinder { radius, length } => {
            // URDF cylinders run along local +z. Upright: a circle; laid
            // over: the hull of both rims.
            let axis = pose.rotation * nalgebra::Vector3::z();
            let c = pose.translation.vector;
            let half = length / 2.0;
            if axis.z.abs() > 0.99 {
                return Some((
                    LayoutShape::Circle {
                        center: [c.x, c.y],
                        radius: *radius,
                    },
                    (c.z - half, c.z + half),
                ));
            }
            let mut pts = Vec::with_capacity(32);
            let mut zmin = f64::INFINITY;
            let mut zmax = f64::NEG_INFINITY;
            for k in 0..16 {
                let a = k as f64 / 16.0 * std::f64::consts::TAU;
                for s in [-1.0, 1.0] {
                    let p = pose * Point3::new(radius * a.cos(), radius * a.sin(), s * half);
                    zmin = zmin.min(p.z);
                    zmax = zmax.max(p.z);
                    pts.push(xy(p));
                }
            }
            Some((
                LayoutShape::Polygon {
                    points: convex_hull(pts),
                },
                (zmin, zmax),
            ))
        }
        Geometry::Mesh { .. } => None,
    }
}

fn shape_points(shape: &LayoutShape) -> Vec<[f64; 2]> {
    match shape {
        LayoutShape::Polygon { points } | LayoutShape::Polyline { points } => points.clone(),
        LayoutShape::Circle { center, radius } => vec![
            [center[0] - radius, center[1] - radius],
            [center[0] + radius, center[1] + radius],
        ],
        LayoutShape::Line { from, to } | LayoutShape::Arrow { from, to } => vec![*from, *to],
        LayoutShape::Text { at, .. } | LayoutShape::Cross { at, .. } => vec![*at],
    }
}

fn centroid(points: &[[f64; 2]]) -> [f64; 2] {
    if points.is_empty() {
        return [0.0, 0.0];
    }
    let n = points.len() as f64;
    let (sx, sy) = points
        .iter()
        .fold((0.0, 0.0), |(x, y), p| (x + p[0], y + p[1]));
    [sx / n, sy / n]
}

fn bbox_center(points: &[[f64; 2]]) -> [f64; 2] {
    let mut fp = Footprint::empty();
    for p in points {
        fp.include(*p);
    }
    if fp.is_empty() {
        return [0.0, 0.0];
    }
    [(fp.min[0] + fp.max[0]) / 2.0, (fp.min[1] + fp.max[1]) / 2.0]
}

/// Shoelace area of an outline (0 for lines and points).
fn polygon_area(points: &[[f64; 2]]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let mut twice = 0.0;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        twice += a[0] * b[1] - b[0] * a[1];
    }
    twice.abs() / 2.0
}

fn last_segment(name: &str) -> &str {
    name.rsplit('/').find(|s| !s.is_empty()).unwrap_or(name)
}

/// One label on the sheet in the making: `(unit key, label, member
/// outlines)` — the outlines decide where the label goes.
type LabelUnit = (String, String, Vec<Vec<[f64; 2]>>);

/// Which name labels an obstacle on the sheet. Flat names label
/// themselves. Hierarchical names (USD prim paths, generated assemblies)
/// label by their *unit*: the first segment below the branch's container
/// chain — so `/World/Conveyor/Belt` and `/World/Conveyor/RollerHead` label
/// once as `Conveyor`, `fence/p0..p11` once as `fence`, and
/// `env/World/Pedestal/Column` as `Pedestal`. A container is a node with
/// at least one non-leaf child (a stage root, an import prefix); a node
/// whose children are all leaves is a thing, not a folder. Parts pinned to
/// obstacles and groups take precedence over this (see [`Scene::layout`]).
struct LabelUnits {
    /// Container-chain depth per top-level branch (segments to skip).
    root_depth: std::collections::BTreeMap<String, usize>,
}

impl LabelUnits {
    fn new(names: &[&str]) -> LabelUnits {
        use std::collections::{BTreeMap, BTreeSet};
        // children[path] = set of child segments; a path with no entry is a leaf.
        let mut children: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        for name in names {
            let segs: Vec<&str> = name.split('/').filter(|s| !s.is_empty()).collect();
            let mut path = String::new();
            for (i, seg) in segs.iter().enumerate() {
                let parent = path.clone();
                if i > 0 {
                    path.push('/');
                }
                path.push_str(seg);
                children.entry(parent).or_default().insert(seg.to_string());
            }
        }
        let is_leaf = |path: &str| !children.contains_key(path);
        let mut root_depth = BTreeMap::new();
        for top in children.get("").cloned().unwrap_or_default() {
            let mut depth = 0;
            let mut node = top.clone();
            loop {
                let kids = children.get(&node).cloned().unwrap_or_default();
                let non_leaf: Vec<String> = kids
                    .iter()
                    .filter(|k| !is_leaf(&format!("{node}/{k}")))
                    .cloned()
                    .collect();
                if non_leaf.is_empty() {
                    break;
                }
                depth += 1;
                if kids.len() == 1 {
                    node = format!("{node}/{}", non_leaf[0]);
                } else {
                    break;
                }
            }
            root_depth.insert(top, depth);
        }
        LabelUnits { root_depth }
    }

    /// `(unit key, label)` for an obstacle name.
    fn unit(&self, name: &str) -> (String, String) {
        let leading = name.starts_with('/');
        let segs: Vec<&str> = name.split('/').filter(|s| !s.is_empty()).collect();
        if segs.len() <= 1 {
            return (name.to_string(), name.to_string());
        }
        let root_depth = self.root_depth.get(segs[0]).copied().unwrap_or(0);
        let take = if segs.len() - root_depth <= 1 {
            segs.len()
        } else {
            (root_depth + 1).min(segs.len() - 1)
        };
        let mut key = String::new();
        if leading {
            key.push('/');
        }
        key.push_str(&segs[..take].join("/"));
        (key, segs[take - 1].to_string())
    }
}

// ------------------------------------------------------------- building

impl Scene {
    /// The plan-view sheet of this scene (see the module docs).
    pub fn layout(&self, options: &LayoutOptions) -> LayoutSheet {
        let mut items: Vec<LayoutItem> = Vec::new();
        let mut footprint = Footprint::empty();
        // (unit key, label, member outlines) — the outlines decide where the
        // label goes: inside a compact thing, above a ring (a fence).
        let mut label_units: Vec<LabelUnit> = Vec::new();

        // Parts pinned to obstacles / groups: their targets are the label
        // units of first resort, labelled by the part.
        let part_targets: Vec<(&str, PartTargetKind, String)> = self
            .parts()
            .iter()
            .filter(|p| matches!(p.kind, PartTargetKind::Obstacle | PartTargetKind::Group))
            .map(|p| {
                let mut label = last_segment(&p.target).to_string();
                if let Some(model) = &p.part.model {
                    label = format!("{label} ({model})");
                } else if let Some(catalog) = &p.part.catalog {
                    label = format!("{label} ({})", catalog.id);
                }
                (p.target.as_str(), p.kind, label)
            })
            .collect();

        let names: Vec<&str> = self.obstacles().iter().map(|o| o.name.as_str()).collect();
        let units = LabelUnits::new(&names);
        let named_equipment: Vec<&str> = self
            .devices()
            .iter()
            .map(|d| d.name.as_str())
            .chain(self.sensors().iter().map(|s| s.name.as_str()))
            .collect();

        // ---- obstacles -------------------------------------------------
        for (i, o) in self.obstacles().iter().enumerate() {
            if !o.visible {
                continue;
            }
            let (shape, (zmin, zmax)) = match primitive_footprint(&o.geometry, &o.pose) {
                Some(f) => f,
                None => match self.obstacle_colliders[i].aabb(&o.pose) {
                    Some((lo, hi)) => (
                        LayoutShape::Polygon {
                            points: vec![
                                [lo[0], lo[1]],
                                [hi[0], lo[1]],
                                [hi[0], hi[1]],
                                [lo[0], hi[1]],
                            ],
                        },
                        (lo[2], hi[2]),
                    ),
                    None => continue,
                },
            };
            let ground = zmax <= options.ground_z;
            let points = shape_points(&shape);
            if !ground {
                for p in &points {
                    footprint.include(*p);
                }
                footprint.height = footprint.height.max(zmax);
            }
            let _ = zmin;
            items.push(LayoutItem {
                layer: if ground {
                    LayoutLayer::Ground
                } else {
                    LayoutLayer::Equipment
                },
                name: o.name.clone(),
                shape,
                dashed: false,
            });
            if ground || !options.labels {
                continue;
            }
            // Which unit labels this obstacle: the *widest* pinned part
            // covering it (a fence's part over its posts' part — the
            // assembly labels once), else its name-derived unit.
            let pinned = part_targets
                .iter()
                .filter(|(target, kind, _)| match kind {
                    PartTargetKind::Obstacle => *target == o.name,
                    PartTargetKind::Group => o.name.starts_with(&format!("{target}/")),
                    _ => false,
                })
                .min_by_key(|(target, _, _)| target.len());
            let (key, label) = match pinned {
                Some((target, _, label)) => (target.to_string(), label.clone()),
                None => units.unit(&o.name),
            };
            // A body generated under a device's or sensor's own name
            // (`conv/belt` for the conveyor `conv`) is labelled by that
            // device — no second label on the geometry. The whole branch
            // counts, not just its first level: a catalog conveyor puts its
            // stands under `conv/stands/…`, which makes `conv` a container
            // and its unit key `belt` — still the conveyor's geometry.
            // Anything with a part of its own was taken above and keeps its
            // label (the stands are their own line on the bill).
            let branch = o
                .name
                .trim_start_matches('/')
                .split('/')
                .next()
                .unwrap_or("");
            if pinned.is_none()
                && (named_equipment.contains(&key.trim_start_matches('/'))
                    || named_equipment.contains(&branch))
            {
                continue;
            }
            match label_units.iter_mut().find(|(k, _, _)| *k == key) {
                Some((_, _, outlines)) => outlines.push(points),
                None => label_units.push((key, label, vec![points])),
            }
        }

        // ---- robots ----------------------------------------------------
        for robot in self.robots() {
            let base = robot.base_pose().translation.vector;
            let at = [base.x, base.y];
            footprint.include(at);
            items.push(LayoutItem {
                layer: LayoutLayer::Robot,
                name: robot.name.clone(),
                shape: LayoutShape::Circle {
                    center: at,
                    radius: 0.12,
                },
                dashed: false,
            });
            if options.labels {
                items.push(LayoutItem {
                    layer: LayoutLayer::Label,
                    name: robot.name.clone(),
                    shape: LayoutShape::Text {
                        at: [at[0], at[1] + 0.2],
                        text: robot.name.clone(),
                        size: 0.12,
                    },
                    dashed: false,
                });
            }
            if options.reach {
                if let Some(reach) = catalog_reach_m(&robot.model.source) {
                    footprint.include([at[0] - reach, at[1] - reach]);
                    footprint.include([at[0] + reach, at[1] + reach]);
                    items.push(LayoutItem {
                        layer: LayoutLayer::Reach,
                        name: robot.name.clone(),
                        shape: LayoutShape::Circle {
                            center: at,
                            radius: reach,
                        },
                        dashed: true,
                    });
                }
            }
        }

        // ---- devices ---------------------------------------------------
        for device in self.devices() {
            match &device.kind {
                DeviceKind::Conveyor {
                    zone_pose,
                    zone_size,
                    velocity,
                    ..
                } => {
                    if let Some((shape, _)) =
                        primitive_footprint(&Geometry::Box { size: *zone_size }, zone_pose)
                    {
                        let pts = shape_points(&shape);
                        for p in &pts {
                            footprint.include(*p);
                        }
                        let c = bbox_center(&pts);
                        items.push(LayoutItem {
                            layer: LayoutLayer::Device,
                            name: device.name.clone(),
                            shape,
                            dashed: true,
                        });
                        let v = velocity.xy();
                        if v.norm() > 1e-9 {
                            let d = v.normalize() * (zone_size.x.min(zone_size.y) * 0.4).max(0.15);
                            items.push(LayoutItem {
                                layer: LayoutLayer::Device,
                                name: device.name.clone(),
                                shape: LayoutShape::Arrow {
                                    from: [c[0] - d.x, c[1] - d.y],
                                    to: [c[0] + d.x, c[1] + d.y],
                                },
                                dashed: false,
                            });
                        }
                        if options.labels {
                            items.push(LayoutItem {
                                layer: LayoutLayer::Label,
                                name: device.name.clone(),
                                shape: LayoutShape::Text {
                                    at: [c[0], c[1] + zone_size.y / 2.0 + 0.1],
                                    text: device.name.clone(),
                                    size: 0.1,
                                },
                                dashed: false,
                            });
                        }
                    }
                }
                DeviceKind::Sink {
                    zone_pose,
                    zone_size,
                    ..
                } => {
                    if let Some((shape, _)) =
                        primitive_footprint(&Geometry::Box { size: *zone_size }, zone_pose)
                    {
                        let pts = shape_points(&shape);
                        let c = bbox_center(&pts);
                        items.push(LayoutItem {
                            layer: LayoutLayer::Device,
                            name: device.name.clone(),
                            shape,
                            dashed: true,
                        });
                        if options.labels {
                            items.push(LayoutItem {
                                layer: LayoutLayer::Label,
                                name: device.name.clone(),
                                shape: LayoutShape::Text {
                                    at: c,
                                    text: device.name.clone(),
                                    size: 0.08,
                                },
                                dashed: false,
                            });
                        }
                    }
                }
                DeviceKind::LinearAxis {
                    objects,
                    axis,
                    range,
                    ..
                } => {
                    // Travel arrow through the driven objects' centre.
                    let mut pts = Vec::new();
                    for name in objects {
                        if let Some((lo, hi)) = self.obstacle_bounds(name) {
                            pts.push([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0]);
                        }
                    }
                    if pts.is_empty() {
                        continue;
                    }
                    let c = centroid(&pts);
                    let a = axis.xy();
                    if a.norm() < 1e-9 {
                        continue;
                    }
                    let a = a.normalize();
                    let (lo, hi) = *range;
                    let from = [c[0] + a.x * lo, c[1] + a.y * lo];
                    let to = [c[0] + a.x * hi, c[1] + a.y * hi];
                    footprint.include(from);
                    footprint.include(to);
                    items.push(LayoutItem {
                        layer: LayoutLayer::Device,
                        name: device.name.clone(),
                        shape: LayoutShape::Arrow { from, to },
                        dashed: false,
                    });
                    if options.labels {
                        items.push(LayoutItem {
                            layer: LayoutLayer::Label,
                            name: device.name.clone(),
                            shape: LayoutShape::Text {
                                at: [to[0], to[1] + 0.1],
                                text: device.name.clone(),
                                size: 0.08,
                            },
                            dashed: false,
                        });
                    }
                }
                DeviceKind::Vehicle { path, .. } => {
                    let mut pts: Vec<[f64; 2]> =
                        path.waypoints.iter().map(|p| [p.x, p.y]).collect();
                    if path.ring && pts.len() > 1 {
                        pts.push(pts[0]);
                    }
                    for p in &pts {
                        footprint.include(*p);
                    }
                    items.push(LayoutItem {
                        layer: LayoutLayer::Device,
                        name: device.name.clone(),
                        shape: LayoutShape::Polyline { points: pts },
                        dashed: true,
                    });
                    for (station, index) in &path.stations {
                        if let Some(p) = path.waypoints.get(*index) {
                            items.push(LayoutItem {
                                layer: LayoutLayer::Device,
                                name: device.name.clone(),
                                shape: LayoutShape::Circle {
                                    center: [p.x, p.y],
                                    radius: 0.08,
                                },
                                dashed: false,
                            });
                            if options.labels {
                                // The layout is a plan view; a station off
                                // the ground plane carries its height in
                                // the label instead of pretending to be on
                                // the floor.
                                let text = if p.z.abs() > 1e-6 {
                                    format!("{}:{station} @z={:.2}", device.name, p.z)
                                } else {
                                    format!("{}:{station}", device.name)
                                };
                                items.push(LayoutItem {
                                    layer: LayoutLayer::Label,
                                    name: device.name.clone(),
                                    shape: LayoutShape::Text {
                                        at: [p.x, p.y + 0.15],
                                        text,
                                        size: 0.08,
                                    },
                                    dashed: false,
                                });
                            }
                        }
                    }
                }
                DeviceKind::Source { .. } => {}
                // The car is ordinary obstacles (already drawn); a plan
                // view has nothing more to say about a vertical ride.
                DeviceKind::Lift { .. } => {}
            }
        }

        // ---- sensors ---------------------------------------------------
        for sensor in self.sensors() {
            match &sensor.kind {
                SensorKind::Zone { pose, size } => {
                    if let Some((shape, _)) =
                        primitive_footprint(&Geometry::Box { size: *size }, pose)
                    {
                        let pts = shape_points(&shape);
                        let c = bbox_center(&pts);
                        items.push(LayoutItem {
                            layer: LayoutLayer::Sensor,
                            name: sensor.name.clone(),
                            shape,
                            dashed: true,
                        });
                        if options.labels {
                            items.push(LayoutItem {
                                layer: LayoutLayer::Label,
                                name: sensor.name.clone(),
                                shape: LayoutShape::Text {
                                    at: c,
                                    text: sensor.name.clone(),
                                    size: 0.08,
                                },
                                dashed: false,
                            });
                        }
                    }
                }
                SensorKind::Vision {
                    camera,
                    detect_range,
                    ..
                } => {
                    // A fixture camera's field of view as a dashed wedge;
                    // a straight-down (or moving) camera has no telling
                    // plan direction, so it draws as its footprint circle.
                    let Some(cam) = self.cameras().iter().find(|c| &c.name == camera) else {
                        continue;
                    };
                    if !matches!(cam.mount, crate::seq::CameraMount::World) {
                        continue;
                    }
                    let far = detect_range.map(|r| r[1]).unwrap_or(cam.far);
                    let half = cam.fov_deg.to_radians() / 2.0;
                    let o = cam.pose.translation;
                    let at = [o.x, o.y];
                    let dir = cam.pose.rotation * nalgebra::Vector3::new(0.0, 0.0, -1.0);
                    let planar = (dir.x * dir.x + dir.y * dir.y).sqrt();
                    if planar > 0.2 {
                        let heading = dir.y.atan2(dir.x);
                        let reach = far * planar;
                        let edge = |s: f64| {
                            let ang = heading + s * half;
                            [at[0] + reach * ang.cos(), at[1] + reach * ang.sin()]
                        };
                        let (left, right) = (edge(-1.0), edge(1.0));
                        for to in [left, right] {
                            items.push(LayoutItem {
                                layer: LayoutLayer::Sensor,
                                name: sensor.name.clone(),
                                shape: LayoutShape::Line { from: at, to },
                                dashed: true,
                            });
                        }
                        items.push(LayoutItem {
                            layer: LayoutLayer::Sensor,
                            name: sensor.name.clone(),
                            shape: LayoutShape::Line {
                                from: left,
                                to: right,
                            },
                            dashed: true,
                        });
                    } else {
                        items.push(LayoutItem {
                            layer: LayoutLayer::Sensor,
                            name: sensor.name.clone(),
                            shape: LayoutShape::Circle {
                                center: at,
                                radius: far * half.tan(),
                            },
                            dashed: true,
                        });
                    }
                    if options.labels {
                        items.push(LayoutItem {
                            layer: LayoutLayer::Label,
                            name: sensor.name.clone(),
                            shape: LayoutShape::Text {
                                at,
                                text: sensor.name.clone(),
                                size: 0.08,
                            },
                            dashed: false,
                        });
                    }
                }
                SensorKind::Beam { from, to, .. } => {
                    let a = [from.x, from.y];
                    let b = [to.x, to.y];
                    items.push(LayoutItem {
                        layer: LayoutLayer::Sensor,
                        name: sensor.name.clone(),
                        shape: LayoutShape::Line { from: a, to: b },
                        dashed: false,
                    });
                    for p in [a, b] {
                        items.push(LayoutItem {
                            layer: LayoutLayer::Sensor,
                            name: sensor.name.clone(),
                            shape: LayoutShape::Circle {
                                center: p,
                                radius: 0.03,
                            },
                            dashed: false,
                        });
                    }
                    if options.labels {
                        items.push(LayoutItem {
                            layer: LayoutLayer::Label,
                            name: sensor.name.clone(),
                            shape: LayoutShape::Text {
                                at: [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0 + 0.08],
                                text: sensor.name.clone(),
                                size: 0.08,
                            },
                            dashed: false,
                        });
                    }
                }
            }
        }

        // ---- frames ----------------------------------------------------
        if options.frames {
            for frame in self.frames() {
                let t = frame.pose.translation.vector;
                let at = [t.x, t.y];
                items.push(LayoutItem {
                    layer: LayoutLayer::Frame,
                    name: frame.name.clone(),
                    shape: LayoutShape::Cross { at, size: 0.08 },
                    dashed: false,
                });
                if options.labels {
                    items.push(LayoutItem {
                        layer: LayoutLayer::Label,
                        name: frame.name.clone(),
                        shape: LayoutShape::Text {
                            at: [at[0], at[1] - 0.1],
                            text: last_segment(&frame.name).to_string(),
                            size: 0.06,
                        },
                        dashed: false,
                    });
                }
            }
        }

        // ---- equipment labels ------------------------------------------
        for (_, label, outlines) in &label_units {
            let all: Vec<[f64; 2]> = outlines.iter().flatten().copied().collect();
            let mut bb = Footprint::empty();
            for p in &all {
                bb.include(*p);
            }
            // A ring (a fence, a guard) covers little of its own bounding
            // box: label it above the top edge, where the enclosure reads
            // as one thing and the label stays off whatever it encloses.
            let covered: f64 = outlines.iter().map(|o| polygon_area(o)).sum();
            let ring = bb.area() > 1e-9 && covered / bb.area() < 0.3 && outlines.len() > 1;
            let at = if ring {
                [(bb.min[0] + bb.max[0]) / 2.0, bb.max[1] + 0.12]
            } else {
                bbox_center(&all)
            };
            items.push(LayoutItem {
                layer: LayoutLayer::Label,
                name: label.clone(),
                shape: LayoutShape::Text {
                    at,
                    text: label.clone(),
                    size: 0.1,
                },
                dashed: false,
            });
        }

        if footprint.is_empty() {
            footprint = Footprint {
                min: [0.0, 0.0],
                max: [0.0, 0.0],
                height: 0.0,
            };
        }

        let title = if options.title.is_empty() {
            self.robots()
                .first()
                .map(|r| format!("{} cell", r.name))
                .unwrap_or_else(|| "cell".to_string())
        } else {
            options.title.clone()
        };
        let mut sheet = LayoutSheet {
            title,
            footprint,
            items,
        };
        sheet.add_decoration(options.grid);
        sheet
    }

    /// The overall plan-view extent of the equipment (ground excluded).
    pub fn footprint(&self, ground_z: f64) -> Footprint {
        let options = LayoutOptions {
            ground_z,
            frames: false,
            labels: false,
            reach: true,
            grid: None,
            title: String::new(),
        };
        self.layout(&options).footprint
    }
}

/// The reach of a catalog robot in metres (`specs.reach_mm`), when the
/// manifest declared one. Composites take their base's.
fn catalog_reach_m(source: &RobotSource) -> Option<f64> {
    match source {
        RobotSource::Catalog { meta, .. } => meta
            .specs
            .iter()
            .find(|(k, _)| k == "reach_mm")
            .map(|(_, v)| v / 1000.0),
        RobotSource::Composite { base, .. } => catalog_reach_m(base),
        _ => None,
    }
}

impl LayoutSheet {
    /// Grid and overall dimensions around the footprint.
    fn add_decoration(&mut self, grid: Option<f64>) {
        let fp = self.footprint;
        if fp.width() <= 0.0 && fp.depth() <= 0.0 {
            return;
        }
        let margin = 0.5;
        let (x0, y0, x1, y1) = (
            fp.min[0] - margin,
            fp.min[1] - margin,
            fp.max[0] + margin,
            fp.max[1] + margin,
        );
        if let Some(pitch) = grid.filter(|p| *p > 0.0) {
            // Grid lines that fall inside the margin box only.
            let gx0 = (x0 / pitch).ceil() * pitch;
            let gy0 = (y0 / pitch).ceil() * pitch;
            let mut x = gx0;
            let mut n = 0;
            while x <= x1 && n < 500 {
                self.items.push(LayoutItem {
                    layer: LayoutLayer::Grid,
                    name: String::new(),
                    shape: LayoutShape::Line {
                        from: [x, y0],
                        to: [x, y1],
                    },
                    dashed: false,
                });
                x += pitch;
                n += 1;
            }
            let mut y = gy0;
            n = 0;
            while y <= y1 && n < 500 {
                self.items.push(LayoutItem {
                    layer: LayoutLayer::Grid,
                    name: String::new(),
                    shape: LayoutShape::Line {
                        from: [x0, y],
                        to: [x1, y],
                    },
                    dashed: false,
                });
                y += pitch;
                n += 1;
            }
        }
        // Overall width along the bottom, depth along the left.
        let dy = fp.min[1] - margin * 0.6;
        let dx = fp.min[0] - margin * 0.6;
        self.items.push(LayoutItem {
            layer: LayoutLayer::Dimension,
            name: String::new(),
            shape: LayoutShape::Line {
                from: [fp.min[0], dy],
                to: [fp.max[0], dy],
            },
            dashed: false,
        });
        self.items.push(LayoutItem {
            layer: LayoutLayer::Dimension,
            name: String::new(),
            shape: LayoutShape::Text {
                at: [(fp.min[0] + fp.max[0]) / 2.0, dy - 0.12],
                text: format!("{:.2} m", fp.width()),
                size: 0.1,
            },
            dashed: false,
        });
        self.items.push(LayoutItem {
            layer: LayoutLayer::Dimension,
            name: String::new(),
            shape: LayoutShape::Line {
                from: [dx, fp.min[1]],
                to: [dx, fp.max[1]],
            },
            dashed: false,
        });
        self.items.push(LayoutItem {
            layer: LayoutLayer::Dimension,
            name: String::new(),
            shape: LayoutShape::Text {
                at: [dx - 0.12, (fp.min[1] + fp.max[1]) / 2.0],
                text: format!("{:.2} m", fp.depth()),
                size: 0.1,
            },
            dashed: false,
        });
    }

    /// The drawn extent in metres — footprint plus the sheet margin (grid,
    /// dimensions and labels live in it). Ground is not counted: a floor
    /// slab is a backdrop, and letting it size the sheet would shrink the
    /// cell to a stamp in the middle of an empty page.
    fn drawn_bounds(&self) -> ([f64; 2], [f64; 2]) {
        let mut fp = Footprint::empty();
        for item in &self.items {
            if item.layer == LayoutLayer::Ground {
                continue;
            }
            for p in shape_points(&item.shape) {
                fp.include(p);
            }
        }
        if fp.is_empty() {
            return ([0.0, 0.0], [1.0, 1.0]);
        }
        (fp.min, fp.max)
    }

    /// Self-contained SVG. `scale` is pixels per metre.
    pub fn to_svg(&self, scale: f64) -> String {
        let scale = if scale.is_finite() && scale > 0.0 {
            scale
        } else {
            100.0
        };
        let (lo, hi) = self.drawn_bounds();
        let pad = 0.4; // metres around the drawn extent, room for the title
        let w_m = (hi[0] - lo[0]) + 2.0 * pad;
        let h_m = (hi[1] - lo[1]) + 2.0 * pad + 0.4;
        let width = (w_m * scale).ceil();
        let height = (h_m * scale).ceil();
        let x = |v: f64| (v - lo[0] + pad) * scale;
        let y = |v: f64| (hi[1] - v + pad + 0.4) * scale;
        let mut out = String::new();
        out.push_str(&format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width}\" height=\"{height}\" viewBox=\"0 0 {width} {height}\" font-family=\"Helvetica, Arial, sans-serif\">\n"
        ));
        out.push_str("<style>\n");
        out.push_str(".ground{fill:#f4f4f2;stroke:#d8d8d4;stroke-width:1}\n");
        out.push_str(".equipment{fill:#dfe6ee;stroke:#3b4756;stroke-width:1.2}\n");
        out.push_str(".robot{fill:#f6c343;stroke:#8a6d00;stroke-width:1.5}\n");
        out.push_str(".reach{fill:none;stroke:#c08a00;stroke-width:1;stroke-dasharray:6 4}\n");
        out.push_str(".device{fill:none;stroke:#2b7de9;stroke-width:1.5}\n");
        out.push_str(".sensor{fill:none;stroke:#d1495b;stroke-width:1.5}\n");
        out.push_str(".frame{fill:none;stroke:#2e8b57;stroke-width:1.2}\n");
        out.push_str(".label{fill:#1e2530;text-anchor:middle;dominant-baseline:middle}\n");
        out.push_str(".dim{fill:#1e2530;stroke:#1e2530;stroke-width:1;text-anchor:middle;dominant-baseline:middle}\n");
        out.push_str(".grid{fill:none;stroke:#ececec;stroke-width:1}\n");
        out.push_str(".title{fill:#1e2530;font-size:14px;font-weight:600}\n");
        out.push_str(".dashed{stroke-dasharray:6 4}\n");
        out.push_str("</style>\n");
        out.push_str(&format!(
            "<rect x=\"0\" y=\"0\" width=\"{width}\" height=\"{height}\" fill=\"#ffffff\"/>\n"
        ));
        // Arrow heads.
        out.push_str("<defs><marker id=\"arrow\" markerWidth=\"8\" markerHeight=\"8\" refX=\"7\" refY=\"4\" orient=\"auto\"><path d=\"M0,0 L8,4 L0,8 z\" fill=\"#2b7de9\"/></marker></defs>\n");
        // Layers back to front.
        for layer in LayoutLayer::ALL {
            let items: Vec<&LayoutItem> = self.items.iter().filter(|i| i.layer == layer).collect();
            if items.is_empty() {
                continue;
            }
            out.push_str(&format!("<g class=\"{}\">\n", layer.svg_class()));
            for item in items {
                let dashed = if item.dashed { " class=\"dashed\"" } else { "" };
                let title = if item.name.is_empty() {
                    String::new()
                } else {
                    format!("<title>{}</title>", svg_escape(&item.name))
                };
                match &item.shape {
                    LayoutShape::Polygon { points } => {
                        let pts: Vec<String> = points
                            .iter()
                            .map(|p| format!("{:.1},{:.1}", x(p[0]), y(p[1])))
                            .collect();
                        out.push_str(&format!(
                            "<polygon{dashed} points=\"{}\">{title}</polygon>\n",
                            pts.join(" ")
                        ));
                    }
                    LayoutShape::Polyline { points } => {
                        let pts: Vec<String> = points
                            .iter()
                            .map(|p| format!("{:.1},{:.1}", x(p[0]), y(p[1])))
                            .collect();
                        out.push_str(&format!(
                            "<polyline{dashed} fill=\"none\" points=\"{}\">{title}</polyline>\n",
                            pts.join(" ")
                        ));
                    }
                    LayoutShape::Circle { center, radius } => out.push_str(&format!(
                        "<circle{dashed} cx=\"{:.1}\" cy=\"{:.1}\" r=\"{:.1}\">{title}</circle>\n",
                        x(center[0]),
                        y(center[1]),
                        radius * scale
                    )),
                    LayoutShape::Line { from, to } => out.push_str(&format!(
                        "<line{dashed} x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\">{title}</line>\n",
                        x(from[0]),
                        y(from[1]),
                        x(to[0]),
                        y(to[1])
                    )),
                    LayoutShape::Arrow { from, to } => out.push_str(&format!(
                        "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" marker-end=\"url(#arrow)\">{title}</line>\n",
                        x(from[0]),
                        y(from[1]),
                        x(to[0]),
                        y(to[1])
                    )),
                    LayoutShape::Cross { at, size } => {
                        let s = size * scale / 2.0;
                        out.push_str(&format!(
                            "<path d=\"M{:.1},{:.1} L{:.1},{:.1} M{:.1},{:.1} L{:.1},{:.1}\">{title}</path>\n",
                            x(at[0]) - s,
                            y(at[1]),
                            x(at[0]) + s,
                            y(at[1]),
                            x(at[0]),
                            y(at[1]) - s,
                            x(at[0]),
                            y(at[1]) + s
                        ));
                    }
                    LayoutShape::Text { at, text, size } => out.push_str(&format!(
                        "<text x=\"{:.1}\" y=\"{:.1}\" font-size=\"{:.1}\">{}</text>\n",
                        x(at[0]),
                        y(at[1]),
                        (size * scale).max(8.0),
                        svg_escape(text)
                    )),
                }
            }
            out.push_str("</g>\n");
        }
        // The title last, so a floor slab that runs off the sheet cannot
        // paint over it.
        out.push_str(&format!(
            "<text class=\"title\" x=\"{:.1}\" y=\"{:.1}\">{} — plan view, {:.2} × {:.2} m</text>\n",
            0.2 * scale,
            0.28 * scale,
            svg_escape(&self.title),
            self.footprint.width(),
            self.footprint.depth()
        ));
        out.push_str("</svg>\n");
        out
    }

    /// A minimal R12 DXF (LINE / POLYLINE / CIRCLE / TEXT on named
    /// layers), in `units` (`"mm"` — the default of most 2D CAD templates
    /// — or `"m"`). Dashed items are told apart by layer (REACH, DEVICE,
    /// SENSOR), not by linetype, so a bare-bones reader still opens it.
    pub fn to_dxf(&self, units: &str) -> String {
        let k = if units == "m" { 1.0 } else { 1000.0 };
        let (lo, hi) = self.drawn_bounds();
        let mut out = String::new();
        let mut push = |code: &str, value: &str| {
            out.push_str(code);
            out.push('\n');
            out.push_str(value);
            out.push('\n');
        };
        let num = |v: f64| -> String {
            if k > 1.0 {
                format!("{:.1}", v * k)
            } else {
                format!("{:.4}", v * k)
            }
        };
        // Header: version and extents.
        push("0", "SECTION");
        push("2", "HEADER");
        push("9", "$ACADVER");
        push("1", "AC1009");
        push("9", "$EXTMIN");
        push("10", &num(lo[0]));
        push("20", &num(lo[1]));
        push("30", "0.0");
        push("9", "$EXTMAX");
        push("10", &num(hi[0]));
        push("20", &num(hi[1]));
        push("30", "0.0");
        push("0", "ENDSEC");
        // Tables: one linetype, one layer per LayoutLayer.
        push("0", "SECTION");
        push("2", "TABLES");
        push("0", "TABLE");
        push("2", "LTYPE");
        push("70", "1");
        push("0", "LTYPE");
        push("2", "CONTINUOUS");
        push("70", "0");
        push("3", "Solid line");
        push("72", "65");
        push("73", "0");
        push("40", "0.0");
        push("0", "ENDTAB");
        push("0", "TABLE");
        push("2", "LAYER");
        push("70", &LayoutLayer::ALL.len().to_string());
        for layer in LayoutLayer::ALL {
            push("0", "LAYER");
            push("2", layer.dxf_name());
            push("70", "0");
            push("62", &layer.dxf_color().to_string());
            push("6", "CONTINUOUS");
        }
        push("0", "ENDTAB");
        push("0", "ENDSEC");
        // Entities.
        push("0", "SECTION");
        push("2", "ENTITIES");
        for item in &self.items {
            let layer = item.layer.dxf_name();
            match &item.shape {
                LayoutShape::Polygon { points } | LayoutShape::Polyline { points } => {
                    let closed = matches!(item.shape, LayoutShape::Polygon { .. });
                    push("0", "POLYLINE");
                    push("8", layer);
                    push("66", "1");
                    push("70", if closed { "1" } else { "0" });
                    for p in points {
                        push("0", "VERTEX");
                        push("8", layer);
                        push("10", &num(p[0]));
                        push("20", &num(p[1]));
                        push("30", "0.0");
                    }
                    push("0", "SEQEND");
                    push("8", layer);
                }
                LayoutShape::Circle { center, radius } => {
                    push("0", "CIRCLE");
                    push("8", layer);
                    push("10", &num(center[0]));
                    push("20", &num(center[1]));
                    push("30", "0.0");
                    push("40", &num(*radius));
                }
                LayoutShape::Line { from, to } | LayoutShape::Arrow { from, to } => {
                    push("0", "LINE");
                    push("8", layer);
                    push("10", &num(from[0]));
                    push("20", &num(from[1]));
                    push("30", "0.0");
                    push("11", &num(to[0]));
                    push("21", &num(to[1]));
                    push("31", "0.0");
                    if let LayoutShape::Arrow { .. } = item.shape {
                        // A small head: two short lines back from the tip.
                        let dx = to[0] - from[0];
                        let dy = to[1] - from[1];
                        let len = (dx * dx + dy * dy).sqrt();
                        if len > 1e-9 {
                            let (ux, uy) = (dx / len, dy / len);
                            let h = (0.08_f64).min(len * 0.4);
                            for side in [-1.0, 1.0] {
                                let bx = to[0] - ux * h + side * (-uy) * h * 0.5;
                                let by = to[1] - uy * h + side * ux * h * 0.5;
                                push("0", "LINE");
                                push("8", layer);
                                push("10", &num(to[0]));
                                push("20", &num(to[1]));
                                push("30", "0.0");
                                push("11", &num(bx));
                                push("21", &num(by));
                                push("31", "0.0");
                            }
                        }
                    }
                }
                LayoutShape::Cross { at, size } => {
                    let s = size / 2.0;
                    for (a, b) in [
                        ([at[0] - s, at[1]], [at[0] + s, at[1]]),
                        ([at[0], at[1] - s], [at[0], at[1] + s]),
                    ] {
                        push("0", "LINE");
                        push("8", layer);
                        push("10", &num(a[0]));
                        push("20", &num(a[1]));
                        push("30", "0.0");
                        push("11", &num(b[0]));
                        push("21", &num(b[1]));
                        push("31", "0.0");
                    }
                }
                LayoutShape::Text { at, text, size } => {
                    push("0", "TEXT");
                    push("8", layer);
                    push("10", &num(at[0]));
                    push("20", &num(at[1]));
                    push("30", "0.0");
                    push("40", &num(*size));
                    push("1", text);
                    // Centred on the insertion point (horizontal 1 =
                    // centre, vertical 2 = middle) — R12 uses codes 72/73
                    // with the alignment point in 11/21.
                    push("72", "1");
                    push("73", "2");
                    push("11", &num(at[0]));
                    push("21", &num(at[1]));
                    push("31", "0.0");
                }
            }
        }
        push("0", "ENDSEC");
        push("0", "EOF");
        out
    }

    /// The sheet as JSON (items in world metres) — for tests and for a
    /// front end that wants to draw it itself.
    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("layout serializes")
    }
}

fn svg_escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::part::Part;
    use crate::seq::{Device, DeviceKind, Sensor, SensorKind, SensorWatch};
    use botrail_model::RobotModel;
    use nalgebra::{Point3, UnitQuaternion, Vector3};
    use std::sync::Arc;

    const URDF: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="l1"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>"#;

    fn scene() -> Scene {
        Scene::new(Arc::new(RobotModel::from_urdf_str(URDF).unwrap()))
    }

    #[test]
    fn hull_of_a_rotated_box_is_its_four_corners() {
        let pose = Isometry3::from_parts(
            Vector3::new(1.0, 2.0, 0.5).into(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_4),
        );
        let (shape, (zmin, zmax)) = primitive_footprint(
            &Geometry::Box {
                size: Vector3::new(2.0, 1.0, 1.0),
            },
            &pose,
        )
        .unwrap();
        let LayoutShape::Polygon { points } = shape else {
            panic!("box footprint is a polygon");
        };
        assert_eq!(points.len(), 4);
        assert!((zmin - 0.0).abs() < 1e-9 && (zmax - 1.0).abs() < 1e-9);
        // A 45° box: the hull's bounding box is (2+1)/√2 wide.
        let mut fp = Footprint::empty();
        for p in &points {
            fp.include(*p);
        }
        assert!((fp.width() - 3.0 / 2f64.sqrt()).abs() < 1e-9);
    }

    #[test]
    fn ground_is_faint_and_outside_the_footprint() {
        let mut scene = scene();
        scene
            .add_obstacle(
                "floor",
                Geometry::Box {
                    size: Vector3::new(20.0, 20.0, 0.05),
                },
                Isometry3::translation(0.0, 0.0, -0.025),
            )
            .unwrap();
        scene
            .add_obstacle(
                "table",
                Geometry::Box {
                    size: Vector3::new(1.0, 0.5, 0.7),
                },
                Isometry3::translation(1.0, 0.0, 0.35),
            )
            .unwrap();
        let sheet = scene.layout(&LayoutOptions::default());
        let floor = sheet.items.iter().find(|i| i.name == "floor").unwrap();
        assert_eq!(floor.layer, LayoutLayer::Ground);
        let fp = sheet.footprint;
        // Table (0.5..1.5 × -0.25..0.25) plus the robot base at the origin.
        assert!((fp.min[0] - 0.0).abs() < 1e-9 && (fp.max[0] - 1.5).abs() < 1e-9);
        assert!((fp.min[1] + 0.25).abs() < 1e-9 && (fp.max[1] - 0.25).abs() < 1e-9);
        assert!((fp.height - 0.7).abs() < 1e-9);
        assert!((fp.area() - 1.5 * 0.5).abs() < 1e-9);
        // The label of a flat-named obstacle is its name.
        assert!(sheet
            .items
            .iter()
            .any(|i| matches!(&i.shape, LayoutShape::Text { text, .. } if text == "table")));
    }

    #[test]
    fn labels_come_from_parts_and_two_segment_units() {
        let mut scene = scene();
        for name in [
            "/World/Conveyor/Belt",
            "/World/Conveyor/RollerHead",
            "/World/Pedestal/Column",
            "/World/Pallet/Board",
        ] {
            scene
                .add_obstacle(
                    name,
                    Geometry::Box {
                        size: Vector3::new(0.3, 0.3, 0.3),
                    },
                    Isometry3::translation(0.0, 0.0, 0.5),
                )
                .unwrap();
        }
        scene
            .set_part(
                "/World/Pedestal",
                None,
                Part {
                    model: Some("PD-500".into()),
                    ..Part::default()
                },
            )
            .unwrap();
        let sheet = scene.layout(&LayoutOptions::default());
        let labels: Vec<&str> = sheet
            .items
            .iter()
            .filter(|i| i.layer == LayoutLayer::Label)
            .filter_map(|i| match &i.shape {
                LayoutShape::Text { text, .. } => Some(text.as_str()),
                _ => None,
            })
            .collect();
        assert!(labels.contains(&"Conveyor"), "{labels:?}");
        assert!(labels.contains(&"Pallet"), "{labels:?}");
        assert!(labels.contains(&"Pedestal (PD-500)"), "{labels:?}");
        assert!(!labels.contains(&"Pedestal"), "{labels:?}");
        assert!(!labels.contains(&"World"), "{labels:?}");
    }

    #[test]
    fn devices_sensors_frames_and_reach_are_drawn() {
        let mut scene = scene();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: Isometry3::translation(1.0, 0.0, 0.5),
                zone_size: Vector3::new(2.0, 0.4, 0.2),
                velocity: Vector3::new(0.2, 0.0, 0.0),
                running: false,
            },
        });
        scene
            .upsert_sensor(Sensor {
                name: "eye".into(),
                kind: SensorKind::Beam {
                    from: Point3::new(1.0, -0.3, 0.5),
                    to: Point3::new(1.0, 0.3, 0.5),
                    radius: 0.01,
                },
                watch: SensorWatch::All,
                mount: None,
            })
            .unwrap();
        scene.add_frame("env/World/mount", Isometry3::translation(0.0, 1.0, 0.0));
        let sheet = scene.layout(&LayoutOptions::default());
        assert!(sheet
            .items
            .iter()
            .any(|i| i.layer == LayoutLayer::Device && i.name == "belt"));
        assert!(sheet
            .items
            .iter()
            .any(|i| matches!(i.shape, LayoutShape::Arrow { .. })));
        assert!(sheet
            .items
            .iter()
            .any(|i| i.layer == LayoutLayer::Sensor && i.name == "eye"));
        assert!(sheet
            .items
            .iter()
            .any(|i| i.layer == LayoutLayer::Frame && i.name == "env/World/mount"));
        // The frame label is the last segment.
        assert!(sheet
            .items
            .iter()
            .any(|i| matches!(&i.shape, LayoutShape::Text { text, .. } if text == "mount")));
        // No catalog reach on a URDF robot; the base mark is there.
        assert!(!sheet.items.iter().any(|i| i.layer == LayoutLayer::Reach));
        assert!(sheet
            .items
            .iter()
            .any(|i| i.layer == LayoutLayer::Robot && i.name == "arm"));
        // Both renderers produce well-formed documents.
        let svg = sheet.to_svg(100.0);
        assert!(svg.starts_with("<svg ") && svg.trim_end().ends_with("</svg>"));
        assert!(svg.contains("<title>belt</title>"));
        let dxf = sheet.to_dxf("mm");
        assert!(dxf.starts_with("0\nSECTION\n2\nHEADER\n"));
        assert!(dxf.trim_end().ends_with("0\nEOF"));
        assert!(
            dxf.contains("\nCIRCLE\n") && dxf.contains("\nPOLYLINE\n") && dxf.contains("\nTEXT\n")
        );
        // Millimetres: the conveyor zone corner at x = 2 m reads 2000.0.
        assert!(dxf.contains("\n2000.0\n"), "{dxf}");
        let dxf_m = sheet.to_dxf("m");
        assert!(dxf_m.contains("\n2.0000\n"));
    }

    #[test]
    fn catalog_reach_becomes_a_circle() {
        let mut model = RobotModel::from_urdf_str(URDF).unwrap();
        let inner = std::mem::replace(&mut model.source, RobotSource::UrdfXml(String::new()));
        model.source = RobotSource::Catalog {
            id: "acme/arm/r1".into(),
            revision: "sha".into(),
            tcp: None,
            flange: None,
            mount: None,
            meta: botrail_model::CatalogMeta {
                specs: vec![("reach_mm".into(), 850.0)],
                ..Default::default()
            },
            inner: Box::new(inner),
        };
        let scene = Scene::new(Arc::new(model));
        let sheet = scene.layout(&LayoutOptions::default());
        let reach = sheet
            .items
            .iter()
            .find(|i| i.layer == LayoutLayer::Reach)
            .unwrap();
        assert!(
            matches!(reach.shape, LayoutShape::Circle { radius, .. } if (radius - 0.85).abs() < 1e-9)
        );
        // The reach widens the footprint.
        assert!((sheet.footprint.width() - 1.7).abs() < 1e-9);
    }

    #[test]
    fn label_units_skip_containers_and_keep_things() {
        let units = LabelUnits::new(&[
            "/World/Floor",
            "/World/Conveyor/Belt",
            "/World/Conveyor/StandInfeed/PostFront",
            "/World/Pallet/Blocks/Block_1",
            "/World/Racking",
            "fence/p0",
            "fence/p1",
            "cell/table/top",
            "cell/table/leg",
            "cell/rack/shelf",
            "env/World/Pedestal/Column",
            "table",
        ]);
        let unit = |n: &str| units.unit(n);
        assert_eq!(
            unit("/World/Conveyor/Belt"),
            ("/World/Conveyor".into(), "Conveyor".into())
        );
        assert_eq!(
            unit("/World/Conveyor/StandInfeed/PostFront"),
            ("/World/Conveyor".into(), "Conveyor".into())
        );
        assert_eq!(
            unit("/World/Pallet/Blocks/Block_1"),
            ("/World/Pallet".into(), "Pallet".into())
        );
        assert_eq!(
            unit("/World/Floor"),
            ("/World/Floor".into(), "Floor".into())
        );
        assert_eq!(
            unit("/World/Racking"),
            ("/World/Racking".into(), "Racking".into())
        );
        assert_eq!(unit("fence/p0"), ("fence".into(), "fence".into()));
        assert_eq!(
            unit("cell/table/top"),
            ("cell/table".into(), "table".into())
        );
        assert_eq!(unit("cell/rack/shelf"), ("cell/rack".into(), "rack".into()));
        assert_eq!(
            unit("env/World/Pedestal/Column"),
            ("env/World/Pedestal".into(), "Pedestal".into())
        );
        assert_eq!(unit("table"), ("table".into(), "table".into()));
    }
}
