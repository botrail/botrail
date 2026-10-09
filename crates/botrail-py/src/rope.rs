//! Small JSON boundary for an explicit post-bake rope pass.
use botrail_rope::{pass::*, shape_bridge::ShapeData, PoseData, RopeLocation};
use serde::Deserialize;

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
    pub fn parse(json: &str) -> Result<RopePass, String> {
        let d: Self = serde_json::from_str(json).map_err(|e| e.to_string())?;
        use botrail_rope::rapier_rope_types::*;
        let mut collision = CollisionSettings::new(d.radius_m);
        collision.friction = d.friction;
        Ok(RopePass {
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
            color: d.color,
        })
    }
}
