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
        Ok(RopePass {
            spec: RopeSpec::new(
                d.name,
                d.points,
                NativeRopeMaterial::new(
                    d.density_kg_m,
                    SpringSettings::new(500.0, 1.0),
                    SpringSettings::new(20.0, 0.8),
                ),
                SamplingSettings::new(d.spacing_m),
                CollisionSettings::new(d.radius_m),
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
        })
    }
}
