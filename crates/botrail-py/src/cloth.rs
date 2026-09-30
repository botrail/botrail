//! Cloth passes as Python declares them (`bt.cloth`): the JSON form of a
//! cloth and its grippers, resolved against a scene into the pass
//! `botrail-cloth` simulates.

use botrail_cloth::pass::{ClothPass, Grip, GripperBinding};
use botrail_cloth::{
    ClothShape, ClothSpec, GarmentLayers, NeckShape, Pad, SeamJoin, TShirtPattern,
};
use botrail_scene::Scene;
use serde::Deserialize;

fn default_spacing() -> f64 {
    0.02
}
fn default_radius() -> f64 {
    0.04
}
fn default_step() -> f64 {
    0.1
}

/// The cloth: what it is cut as, where it lies, and its shell.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClothDecl {
    pub name: String,
    /// `tshirt` or `sheet`.
    pub kind: String,
    /// A sheet's `(width, length)`.
    #[serde(default)]
    pub size: Option<[f64; 2]>,
    #[serde(default = "default_spacing")]
    pub spacing: f64,
    /// A T-shirt's panels: `sewn` (front and back) or `single`.
    #[serde(default)]
    pub layers: Option<String>,
    /// `notch` or `round`.
    #[serde(default)]
    pub neck: Option<String>,
    /// `shared` or `stitched`.
    #[serde(default)]
    pub seams: Option<String>,
    /// A T-shirt's body `(width, length)`.
    #[serde(default)]
    pub body: Option<[f64; 2]>,
    /// A T-shirt's sleeve `(length, width)`.
    #[serde(default)]
    pub sleeve: Option<[f64; 2]>,
    /// The frame the cloth lies on: its origin is the cloth's centre, its
    /// height the support surface, its heading added to `yaw`.
    #[serde(default)]
    pub on: Option<String>,
    /// The cloth's centre on a support at that height, without a frame.
    #[serde(default)]
    pub position: Option<[f64; 3]>,
    #[serde(default)]
    pub yaw: f64,
    #[serde(default)]
    pub thickness: Option<f64>,
    #[serde(default)]
    pub band: Option<f64>,
    #[serde(default)]
    pub friction: Option<f64>,
    /// Areal density, kg/m².
    #[serde(default)]
    pub density: Option<f64>,
    #[serde(default)]
    pub workers: Option<usize>,
    #[serde(default)]
    pub max_iterations: Option<usize>,
}

/// A box a gripper pushes cloth with, in its tool frame.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PadDecl {
    /// Full extents.
    pub size: [f64; 3],
    #[serde(default)]
    pub offset: [f64; 3],
}

/// One gripper: the signal that closes it and the link it rides.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GripperDecl {
    pub signal: String,
    /// Robot instance; the sole robot when omitted.
    #[serde(default)]
    pub robot: Option<String>,
    /// The arm whose tip is the tool point (a dual-arm robot's group).
    #[serde(default)]
    pub group: Option<String>,
    /// The link whose frame is the tool point; the robot's TCP otherwise.
    #[serde(default)]
    pub link: Option<String>,
    #[serde(default = "default_radius")]
    pub radius: f64,
    #[serde(default)]
    pub compliance: f64,
    /// Cloth steps the hold softens over before it opens.
    #[serde(default)]
    pub soften: usize,
    #[serde(default)]
    pub pad: Option<PadDecl>,
}

/// A cloth and the grippers that handle it.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PassDecl {
    pub cloth: ClothDecl,
    #[serde(default)]
    pub grippers: Vec<GripperDecl>,
    #[serde(default = "default_step")]
    pub step: f64,
}

fn one_of<T: Copy>(what: &str, value: Option<&str>, options: &[(&str, T)]) -> Result<T, String> {
    let Some(value) = value else {
        return Ok(options[0].1);
    };
    options
        .iter()
        .find(|(name, _)| *name == value)
        .map(|(_, v)| *v)
        .ok_or_else(|| {
            let names: Vec<&str> = options.iter().map(|(name, _)| *name).collect();
            format!("unknown {what} `{value}` (one of: {})", names.join(", "))
        })
}

impl PassDecl {
    pub fn parse(json: &str) -> Result<Self, String> {
        serde_json::from_str(json).map_err(|e| format!("cloth: {e}"))
    }

    pub fn name(&self) -> &str {
        &self.cloth.name
    }

    /// The cloth's spec and the height of the surface it lies on, read
    /// against `scene`'s frames.
    pub fn spec(&self, scene: &Scene) -> Result<(ClothSpec, Option<f64>), String> {
        let c = &self.cloth;
        let mut spec = ClothSpec::default();
        if let Some(v) = c.thickness {
            spec.thickness = v;
        }
        if let Some(v) = c.band {
            spec.band = v;
        }
        if let Some(v) = c.friction {
            spec.friction = v;
        }
        if let Some(v) = c.density {
            spec.surface_density = v;
        }
        if let Some(v) = c.workers {
            spec.workers = v;
        }
        if let Some(v) = c.max_iterations {
            spec.max_iterations = v;
        }
        spec.shape = match c.kind.as_str() {
            "tshirt" => {
                if c.size.is_some() {
                    return Err(
                        "`size` is a sheet's; a T-shirt takes `body` and `sleeve`".to_string()
                    );
                }
                let mut pattern = TShirtPattern {
                    spacing: c.spacing,
                    ..TShirtPattern::default()
                };
                if let Some([width, length]) = c.body {
                    pattern.body_width = width;
                    pattern.body_length = length;
                }
                if let Some([length, width]) = c.sleeve {
                    pattern.sleeve_length = length;
                    pattern.sleeve_width = width;
                }
                pattern.layers = one_of(
                    "layers",
                    c.layers.as_deref(),
                    &[
                        ("sewn", GarmentLayers::Sewn),
                        ("single", GarmentLayers::Single),
                    ],
                )?;
                pattern.neck = one_of(
                    "neck",
                    c.neck.as_deref(),
                    &[("notch", NeckShape::Notch), ("round", NeckShape::Round)],
                )?;
                pattern.seams = one_of(
                    "seams",
                    c.seams.as_deref(),
                    &[
                        ("shared", SeamJoin::Shared),
                        ("stitched", SeamJoin::Stitched),
                    ],
                )?;
                // The layers start this far apart, which is what a stitch spans.
                pattern.stitch_length = spec.thickness + spec.band;
                ClothShape::TShirt(pattern)
            }
            "sheet" => ClothShape::Sheet {
                size: c
                    .size
                    .ok_or_else(|| "a sheet needs `size=(width, length)`".to_string())?,
                spacing: c.spacing,
            },
            other => {
                return Err(format!(
                    "unknown cloth kind `{other}` (one of: tshirt, sheet)"
                ))
            }
        };
        let table_top = match (&c.on, c.position) {
            (Some(frame), None) => {
                let pose = scene
                    .frame(frame)
                    .ok_or_else(|| format!("unknown frame `{frame}`"))?
                    .pose;
                let t = pose.translation.vector;
                spec.origin = [t.x, t.y, t.z];
                spec.yaw = c.yaw + pose.rotation.euler_angles().2;
                t.z
            }
            (None, Some(position)) => {
                spec.origin = position;
                spec.yaw = c.yaw;
                position[2]
            }
            (Some(_), Some(_)) => {
                return Err("give the cloth `on` or `position`, not both".to_string())
            }
            (None, None) => {
                return Err(
                    "a cloth needs `on=<frame>` or `position=(x, y, z)` to lie on".to_string(),
                )
            }
        };
        Ok((spec, Some(table_top)))
    }

    /// The pass against `scene`: names resolved to robots, links and frames.
    pub fn resolve(&self, scene: &Scene) -> Result<ClothPass, String> {
        let (spec, table_top) = self.spec(scene)?;
        let robots = scene.robots();
        let mut grippers = Vec::with_capacity(self.grippers.len());
        for g in &self.grippers {
            let robot = match &g.robot {
                Some(name) => scene
                    .robot_index(name)
                    .ok_or_else(|| format!("unknown robot `{name}`"))?,
                None if robots.len() == 1 => 0,
                None => {
                    return Err(format!(
                    "the scene has {} robots; say which one carries the gripper on `{}` (robot=)",
                    robots.len(),
                    g.signal
                ))
                }
            };
            let model = &robots[robot].model;
            let link = match (&g.link, &g.group) {
                (Some(link), _) => model.link_index(link).ok_or_else(|| {
                    format!("robot `{}` has no link `{link}`", robots[robot].name)
                })?,
                (None, Some(group)) => {
                    let index = model.group_index(group).ok_or_else(|| {
                        format!("robot `{}` has no group `{group}`", robots[robot].name)
                    })?;
                    scene.group_tip(robot, Some(index))
                }
                (None, None) => model.default_tcp_link(),
            };
            grippers.push(GripperBinding {
                robot,
                link,
                signal: g.signal.clone(),
                grip: Grip {
                    radius: g.radius,
                    compliance: g.compliance,
                    soften_steps: g.soften,
                    pad: g.pad.as_ref().map(|pad| Pad {
                        half_extents: pad.size.map(|s| 0.5 * s),
                        offset: pad.offset,
                    }),
                },
            });
        }
        Ok(ClothPass {
            name: self.cloth.name.clone(),
            spec,
            table_top,
            grippers,
            step: self.step,
        })
    }
}
