//! Small JSON boundary for an explicit post-bake rope pass.
use botrail_rope::{
    pass::*, shape::StaticSettings, shape_bridge::ShapeData, PoseData, RopeLocation,
};
use serde::Deserialize;

/// How a pass gets its shapes: the replay's dynamics, or the static solve
/// (the rod at rest between its holders at every frame).
pub enum Solver {
    Physics,
    Static(StaticSettings),
}
/// A parsed pass and the solver it asks for.
pub struct Parsed {
    pub pass: RopePass,
    pub solver: Solver,
}

fn default_step() -> f64 {
    1.0 / 240.0
}
fn default_spacing() -> f64 {
    0.02
}
fn default_radius() -> f64 {
    0.005
}
fn default_density() -> f64 {
    0.1
}
fn default_axial_hz() -> f64 {
    500.0
}
fn default_bending_hz() -> f64 {
    20.0
}
fn default_friction() -> f64 {
    0.5
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Decl {
    name: String,
    points: Vec<[f64; 3]>,
    #[serde(default = "default_step")]
    step_s: f64,
    #[serde(default = "default_spacing")]
    spacing_m: f64,
    #[serde(default = "default_radius")]
    radius_m: f64,
    #[serde(default = "default_density")]
    density_kg_m: f64,
    #[serde(default)]
    grippers: Vec<GripDecl>,
    #[serde(default)]
    collision_links: Vec<LinkDecl>,
    #[serde(default)]
    obstacles: Vec<String>,
    #[serde(default)]
    pins: Vec<(RopeLocation, [f64; 3])>,
    #[serde(default)]
    connectors: Vec<ConnectorDecl>,
    #[serde(default)]
    anchors: Vec<AnchorDecl>,
    #[serde(default)]
    guides: Vec<GuideDecl>,
    /// Native spring frequencies (not calibrated EA/EI) and the one
    /// friction coefficient of the rope's collider.
    #[serde(default = "default_axial_hz")]
    axial_hz: f64,
    #[serde(default = "default_bending_hz")]
    bending_hz: f64,
    #[serde(default = "default_friction")]
    friction: f64,
    #[serde(default)]
    color: Option<[f32; 3]>,
    #[serde(default)]
    ground_z: Option<f64>,
    #[serde(default)]
    preroll_s: f64,
    #[serde(default)]
    extra_colliders: Vec<(PoseData, ShapeData)>,
    /// What the bending springs rest at: `"start"` (the shape the rope is
    /// laid in) or `"straight"` (a tube or cable off the reel).
    #[serde(default)]
    rest: Option<String>,
    /// The rope as a rod (Hz): windows of particles each drawn toward
    /// their rest shape by a spring of this frequency; zero for none.
    #[serde(default)]
    rod_hz: f64,
    /// `"physics"` (the replay, the default) or `"static"` (the rod at
    /// rest between its holders at every frame, no dynamics).
    #[serde(default)]
    solver: Option<String>,
    /// The static solve's flexural rigidity (N·m²); a polyurethane
    /// tube's for the line's radius when absent.
    #[serde(default)]
    rigidity_nm2: Option<f64>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LinkDecl {
    robot: String,
    link: String,
}
impl LinkDecl {
    fn binding(self) -> LinkBinding {
        LinkBinding {
            robot: self.robot,
            link: self.link,
        }
    }
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GripDecl {
    robot: String,
    link: String,
    signal: String,
    location: RopeLocation,
}
/// An obstacle, or a robot's link, holding a span of the rope.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct AnchorDecl {
    #[serde(default)]
    obstacle: Option<String>,
    #[serde(default)]
    robot: Option<String>,
    #[serde(default)]
    link: Option<String>,
    location: RopeLocation,
    #[serde(default)]
    length_m: f64,
    #[serde(default)]
    rail: Option<RailDecl>,
    #[serde(default)]
    roll: bool,
}
/// A retracting anchor's rail.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RailDecl {
    axis: [f64; 3],
    stroke_m: f64,
    force_n: f64,
}
impl AnchorDecl {
    fn anchor(self) -> Result<Anchor, String> {
        let body = match (self.obstacle, self.robot, self.link) {
            (Some(obstacle), None, None) => AnchorBody::Obstacle(obstacle),
            (None, Some(robot), Some(link)) => AnchorBody::Link(LinkBinding { robot, link }),
            _ => return Err("an anchor names an obstacle, or a robot and its link".into()),
        };
        Ok(Anchor {
            body,
            location: self.location,
            length_m: self.length_m,
            rail: self.rail.map(|r| Rail {
                axis: r.axis,
                stroke_m: r.stroke_m,
                force_n: r.force_n,
            }),
            roll: self.roll,
        })
    }
}
/// A ring on an obstacle, or a robot's link, the line runs through.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GuideDecl {
    #[serde(default)]
    obstacle: Option<String>,
    #[serde(default)]
    robot: Option<String>,
    #[serde(default)]
    link: Option<String>,
    position: [f64; 3],
    axis: [f64; 3],
    length_m: f64,
    inner_m: f64,
}
impl GuideDecl {
    fn guide(self) -> Result<Guide, String> {
        let body = match (self.obstacle, self.robot, self.link) {
            (Some(obstacle), None, None) => AnchorBody::Obstacle(obstacle),
            (None, Some(robot), Some(link)) => AnchorBody::Link(LinkBinding { robot, link }),
            _ => return Err("a guide names an obstacle, or a robot and its link".into()),
        };
        Ok(Guide {
            body,
            position: self.position,
            axis: self.axis,
            length_m: self.length_m,
            inner_radius_m: self.inner_m / 2.0,
        })
    }
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ConnectorDecl {
    name: String,
    location: RopeLocation,
    shape: ShapeData,
    pose: PoseData,
    mass_kg: f64,
}
impl Decl {
    pub fn parse(json: &str) -> Result<Parsed, String> {
        let d: Self = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let straight_rest = match d.rest.as_deref() {
            None | Some("start") => false,
            Some("straight") => true,
            Some(other) => return Err(format!("rest is \"start\" or \"straight\", not {other:?}")),
        };
        let settings = StaticSettings::new(
            d.rigidity_nm2
                .unwrap_or_else(|| StaticSettings::tube_rigidity(d.radius_m)),
        );
        let solver = match d.solver.as_deref() {
            None | Some("physics") => Solver::Physics,
            Some("static") => Solver::Static(settings),
            Some(other) => {
                return Err(format!(
                    "solver is \"physics\" or \"static\", not {other:?}"
                ))
            }
        };
        if let Some(r) = d.rigidity_nm2 {
            if !(r.is_finite() && r > 0.0) {
                return Err(format!("rigidity_nm2 must be finite and positive, not {r}"));
            }
        }
        use botrail_rope::rapier_rope_types::*;
        let mut collision = CollisionSettings::new(d.radius_m);
        collision.friction = d.friction;
        let pass = RopePass {
            spec: RopeSpec::new(
                d.name,
                d.points,
                NativeRopeMaterial::new(
                    d.density_kg_m,
                    SpringSettings::new(d.axial_hz, 1.0),
                    SpringSettings::new(d.bending_hz, 0.8),
                ),
                SamplingSettings::new(d.spacing_m),
                collision,
            ),
            step_s: d.step_s,
            grippers: d
                .grippers
                .into_iter()
                .map(|g| GripperBinding {
                    link: LinkBinding {
                        robot: g.robot,
                        link: g.link,
                    },
                    signal: g.signal,
                    location: g.location,
                })
                .collect(),
            collision_links: d
                .collision_links
                .into_iter()
                .map(LinkDecl::binding)
                .collect(),
            obstacles: d.obstacles,
            pins: d.pins,
            connectors: d
                .connectors
                .into_iter()
                .map(|c| Connector {
                    name: c.name,
                    location: c.location,
                    shape: c.shape,
                    pose: c.pose,
                    mass_kg: c.mass_kg,
                })
                .collect(),
            anchors: d
                .anchors
                .into_iter()
                .map(AnchorDecl::anchor)
                .collect::<Result<_, _>>()?,
            guides: d
                .guides
                .into_iter()
                .map(GuideDecl::guide)
                .collect::<Result<_, _>>()?,
            color: d.color,
            ground_z: d.ground_z,
            preroll_s: d.preroll_s,
            extra_colliders: d.extra_colliders,
            straight_rest,
            rod_hz: d.rod_hz,
        };
        Ok(Parsed { pass, solver })
    }
}

/// One pass against the bake with the solver it asks for: the replay's
/// track, or the static shapes.
pub fn run(
    scene: &botrail_scene::Scene,
    timeline: &botrail_scene::rollout::SequenceTimeline,
    parsed: &Parsed,
) -> Result<botrail_rope::RopeTrack, String> {
    match &parsed.solver {
        Solver::Physics => botrail_rope::pass::animate(scene, timeline, &parsed.pass)
            .map(|replay| replay.track)
            .map_err(|e| e.to_string()),
        Solver::Static(settings) => {
            botrail_rope::shape::animate(scene, timeline, &parsed.pass, settings)
                .map_err(|e| e.to_string())
        }
    }
}

/// A unit of work: one pass on its own, or static passes solved together.
pub enum WorkItem {
    One(usize),
    Together(Vec<usize>),
}

/// How far apart two laid lines may be and still be solved together (m):
/// lines that could sag or swing into one another.
const TOGETHER_M: f64 = 0.15;

/// The passes grouped for the solve: static passes whose laid lines' boxes
/// come within `TOGETHER_M` of each other (transitively) go together.
pub fn work_items(passes: &[Result<Parsed, String>]) -> Vec<WorkItem> {
    let mut items = Vec::new();
    let mut statics: Vec<usize> = Vec::new();
    for (i, p) in passes.iter().enumerate() {
        match p {
            Ok(parsed) if matches!(parsed.solver, Solver::Static(_)) => statics.push(i),
            _ => items.push(WorkItem::One(i)),
        }
    }
    let bounds = |i: usize| -> ([f64; 3], [f64; 3]) {
        let Ok(parsed) = &passes[i] else {
            return ([0.0; 3], [0.0; 3]);
        };
        let r = parsed.pass.spec.collision.radius_m + TOGETHER_M / 2.0;
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in &parsed.pass.spec.reference_centerline_m {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k] - r);
                hi[k] = hi[k].max(p[k] + r);
            }
        }
        (lo, hi)
    };
    let boxes: Vec<_> = statics.iter().map(|&i| bounds(i)).collect();
    let mut parent: Vec<usize> = (0..statics.len()).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    for a in 0..statics.len() {
        for b in a + 1..statics.len() {
            let (la, ha) = boxes[a];
            let (lb, hb) = boxes[b];
            if (0..3).all(|k| la[k] <= hb[k] && lb[k] <= ha[k]) {
                let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
                if ra != rb {
                    parent[ra] = rb;
                }
            }
        }
    }
    let mut groups: std::collections::BTreeMap<usize, Vec<usize>> = Default::default();
    for (k, &i) in statics.iter().enumerate() {
        let r = root(&mut parent, k);
        groups.entry(r).or_default().push(i);
    }
    for (_, members) in groups {
        items.push(if members.len() == 1 {
            WorkItem::One(members[0])
        } else {
            WorkItem::Together(members)
        });
    }
    items
}

/// One unit of work run: the tracks (or the reasons) per pass index.
pub fn run_item(
    scene: &botrail_scene::Scene,
    timeline: &botrail_scene::rollout::SequenceTimeline,
    passes: &[Result<Parsed, String>],
    item: &WorkItem,
) -> Vec<(usize, Result<botrail_rope::RopeTrack, String>)> {
    match item {
        WorkItem::One(i) => vec![(
            *i,
            match &passes[*i] {
                Ok(parsed) => run(scene, timeline, parsed),
                Err(e) => Err(e.clone()),
            },
        )],
        WorkItem::Together(members) => {
            let together: Vec<(&RopePass, &StaticSettings)> = members
                .iter()
                .filter_map(|&i| match &passes[i] {
                    Ok(Parsed {
                        pass,
                        solver: Solver::Static(settings),
                    }) => Some((pass, settings)),
                    _ => None,
                })
                .collect();
            match botrail_rope::shape::animate_many(scene, timeline, &together) {
                Ok(tracks) => members
                    .iter()
                    .copied()
                    .zip(tracks.into_iter().map(Ok))
                    .collect(),
                Err(e) => members.iter().map(|&i| (i, Err(e.to_string()))).collect(),
            }
        }
    }
}
