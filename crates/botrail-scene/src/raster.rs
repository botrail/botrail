//! A CPU rasteriser for depth and segmentation observations
//! (design-rl-sensors.md RS1).
//!
//! The picture a `seq::Camera` takes of the world as a live rollout
//! stands: the same pinhole (horizontal fov, `-Z` view direction, `+Y`
//! image-up) and the same depth semantics as the studio's depth capture
//! (RealSense z16: Z along the optical axis in meters, `0` for no return
//! or outside `[near, far]`, row 0 the top of the picture), so a policy
//! trains on the camera the studio evaluates it with. Triangles come from
//! the scene's own geometry — visual shapes by default, collision shapes
//! on request — tessellated once per world and posed per frame; a
//! z-buffer over screen-space edge functions keeps every frame
//! bit-identical for the same world.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, OnceLock, RwLock};

use botrail_model::{Geometry, Shape};
use nalgebra::{Isometry3, Point3, Vector3};
use serde::Deserialize;

use crate::rollout::WorldView;
use crate::seq::Camera;
use crate::Scene;

/// Which of a body's shapes the picture is taken of.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum RenderGeometry {
    /// The visual shapes — what the studio draws.
    #[default]
    Visual,
    /// The collision shapes — what planning and physics see.
    Collision,
}

impl RenderGeometry {
    pub fn index(self) -> usize {
        match self {
            RenderGeometry::Visual => 0,
            RenderGeometry::Collision => 1,
        }
    }
}

/// Who a triangle belongs to.
#[derive(Debug, Clone, Copy)]
enum Owner {
    Obstacle(usize),
    Link { robot: usize, link: usize },
}

/// One body's triangles in its own frame, its segmentation id, and the
/// colour its shapes name (a link's visual colour; an obstacle's is read
/// at render time, it may be re-coloured between episodes).
struct Body {
    owner: Owner,
    id: i32,
    tris: Arc<Vec<[Point3<f64>; 3]>>,
    color: Option<[f32; 3]>,
}

/// The light a flat-shaded picture is lit by (design-rl-sensors.md RS2):
/// one directional light plus an ambient floor, Lambert, two-sided, no
/// shadows — albedo × (ambient + (1 − ambient)·|n·l|). The picture is
/// colour by displayColor and shape, not photoreal: the studio and the
/// USD export carry the look.
#[derive(Debug, Clone, PartialEq)]
pub struct Lighting {
    /// Unit direction *toward* the light, world frame.
    pub direction: Vector3<f64>,
    /// Ambient share in `[0, 1]`.
    pub ambient: f64,
    /// Cast shadows (design-rl-sensors.md RS3): an orthographic shadow
    /// map is rendered from the light every frame and a lit pixel
    /// behind another body drops to the ambient share. Hard-edged, no
    /// penumbra; off by default (it costs about a second picture).
    pub shadows: bool,
    /// Shadow-map size, texels a side; `0` (the default) fits it to the
    /// picture — twice its larger side, 64 to 512.
    pub shadow_map: usize,
}

impl Default for Lighting {
    fn default() -> Self {
        Lighting {
            direction: Vector3::new(0.3, -0.5, 0.8).normalize(),
            ambient: 0.35,
            shadows: false,
            shadow_map: 0,
        }
    }
}

impl Lighting {
    /// A lighting from any direction vector (normalised here) and an
    /// ambient share (clamped to `[0, 1]`), without shadows.
    pub fn new(direction: Vector3<f64>, ambient: f64) -> Self {
        let n = direction.norm();
        Lighting {
            direction: if n > 1e-12 {
                direction / n
            } else {
                Lighting::default().direction
            },
            ambient: ambient.clamp(0.0, 1.0),
            ..Lighting::default()
        }
    }

    /// The same light, casting shadows (or not).
    pub fn with_shadows(mut self, on: bool) -> Self {
        self.shadows = on;
        self
    }
}

/// Vertex-clustering decimation: every vertex snaps to the centroid of
/// its `cell`-sized grid cell and triangles that collapse are dropped —
/// a cheap cut of a dense visual mesh's triangle count for pictures a
/// policy reads at 64 pixels a side.
pub fn decimate(tris: &[[Point3<f64>; 3]], cell: f64) -> Vec<[Point3<f64>; 3]> {
    if !cell.is_finite() || cell <= 0.0 {
        return tris.to_vec();
    }
    let key = |p: &Point3<f64>| -> (i64, i64, i64) {
        (
            (p.x / cell).floor() as i64,
            (p.y / cell).floor() as i64,
            (p.z / cell).floor() as i64,
        )
    };
    let mut cells: HashMap<(i64, i64, i64), (Vector3<f64>, f64)> = HashMap::new();
    for tri in tris {
        for p in tri {
            let e = cells.entry(key(p)).or_insert((Vector3::zeros(), 0.0));
            e.0 += p.coords;
            e.1 += 1.0;
        }
    }
    let rep = |p: &Point3<f64>| -> Point3<f64> {
        let (sum, n) = cells[&key(p)];
        Point3::from(sum / n)
    };
    tris.iter()
        .filter(|tri| {
            let (a, b, c) = (key(&tri[0]), key(&tri[1]), key(&tri[2]));
            a != b && b != c && a != c
        })
        .map(|tri| [rep(&tri[0]), rep(&tri[1]), rep(&tri[2])])
        .collect()
}

/// What an uncoloured obstacle is painted (the studio's neutral grey).
const OBSTACLE_ALBEDO: [f32; 3] = [0.7, 0.7, 0.72];
/// What an uncoloured link is painted.
const LINK_ALBEDO: [f32; 3] = [0.75, 0.75, 0.78];

/// The world's triangles, tessellated once; poses are read per frame.
pub struct RenderScene {
    geometry: RenderGeometry,
    bodies: Vec<Body>,
}

/// One rendered picture: row-major, row 0 at the top.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub width: usize,
    pub height: usize,
    /// Z along the optical axis, meters; `0` = no return / outside
    /// `[near, far]`.
    pub depth: Vec<f32>,
    /// Segmentation id per pixel: `0` background, obstacle `i` → `1 + i`,
    /// then robot links in scene order (see [`segmentation_ids`]).
    pub id: Vec<i32>,
    /// Flat-shaded colour per pixel, `r, g, b` in `0..=255`, black for
    /// background — only when the picture was rendered with a
    /// [`Lighting`].
    pub rgb: Option<Vec<u8>>,
}

/// Pinhole intrinsics for a picture of `width × height` pixels with
/// horizontal field of view `fov_deg`: `fx = fy = (w/2) / tan(fov/2)`,
/// principal point at the centre — the studio's depth-capture K.
pub fn intrinsics(fov_deg: f64, width: usize, height: usize) -> (f64, f64, f64, f64) {
    let fx = (width as f64 / 2.0) / (fov_deg.to_radians() / 2.0).tan();
    (fx, fx, width as f64 / 2.0, height as f64 / 2.0)
}

/// The segmentation id of every body a picture can show, by name
/// (obstacles as is, links as `robot/link`); background is `0`.
pub fn segmentation_ids(scene: &Scene) -> Vec<(String, i32)> {
    let mut out = Vec::new();
    for (i, o) in scene.obstacles().iter().enumerate() {
        out.push((o.name.clone(), 1 + i as i32));
    }
    let mut next = 1 + scene.obstacles().len() as i32;
    for robot in scene.robots() {
        for link in &robot.model.links {
            out.push((format!("{}/{}", robot.name, link.name), next));
            next += 1;
        }
    }
    out
}

/// Path, scale bits, decimation cell bits (0: none).
type MeshKey = (PathBuf, [u64; 3], u64);
type Tris = Arc<Vec<[Point3<f64>; 3]>>;
type MeshCache = RwLock<HashMap<MeshKey, Tris>>;

/// Tessellated meshes by path and scale, shared across worlds (32 worlds
/// of one cell load each mesh once).
fn mesh_cache() -> &'static MeshCache {
    static CACHE: OnceLock<MeshCache> = OnceLock::new();
    CACHE.get_or_init(Default::default)
}

fn mesh_tris(path: &std::path::Path, scale: &Vector3<f64>, cell: Option<f64>) -> Option<Tris> {
    let key = (
        path.to_path_buf(),
        [scale.x.to_bits(), scale.y.to_bits(), scale.z.to_bits()],
        cell.map(f64::to_bits).unwrap_or(0),
    );
    if let Some(hit) = mesh_cache().read().expect("mesh cache poisoned").get(&key) {
        return Some(hit.clone());
    }
    let data = botrail_collide::mesh::load_mesh_data(path, scale).ok()?;
    let mut tris: Vec<[Point3<f64>; 3]> = data
        .indices
        .iter()
        .filter_map(|[a, b, c]| {
            let v = |i: u32| {
                data.vertices
                    .get(i as usize)
                    .map(|p| Point3::new(p[0], p[1], p[2]))
            };
            Some([v(*a)?, v(*b)?, v(*c)?])
        })
        .collect();
    if let Some(cell) = cell {
        tris = decimate(&tris, cell);
    }
    let tris = Arc::new(tris);
    mesh_cache()
        .write()
        .expect("mesh cache poisoned")
        .insert(key, tris.clone());
    Some(tris)
}

/// Triangles of one geometry in its own frame (meshes decimated to
/// `cell` when given; primitives are already few triangles).
fn tessellate(geometry: &Geometry, cell: Option<f64>) -> Option<Arc<Vec<[Point3<f64>; 3]>>> {
    let mut tris = Vec::new();
    match geometry {
        Geometry::Box { size } => {
            let (x, y, z) = (size.x / 2.0, size.y / 2.0, size.z / 2.0);
            let p = |sx: f64, sy: f64, sz: f64| Point3::new(sx * x, sy * y, sz * z);
            let quad = |tris: &mut Vec<[Point3<f64>; 3]>,
                        a: Point3<f64>,
                        b: Point3<f64>,
                        c: Point3<f64>,
                        d: Point3<f64>| {
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            };
            quad(
                &mut tris,
                p(-1., -1., 1.),
                p(1., -1., 1.),
                p(1., 1., 1.),
                p(-1., 1., 1.),
            ); // +z
            quad(
                &mut tris,
                p(-1., -1., -1.),
                p(-1., 1., -1.),
                p(1., 1., -1.),
                p(1., -1., -1.),
            ); // -z
            quad(
                &mut tris,
                p(1., -1., -1.),
                p(1., 1., -1.),
                p(1., 1., 1.),
                p(1., -1., 1.),
            ); // +x
            quad(
                &mut tris,
                p(-1., -1., -1.),
                p(-1., -1., 1.),
                p(-1., 1., 1.),
                p(-1., 1., -1.),
            ); // -x
            quad(
                &mut tris,
                p(-1., 1., -1.),
                p(-1., 1., 1.),
                p(1., 1., 1.),
                p(1., 1., -1.),
            ); // +y
            quad(
                &mut tris,
                p(-1., -1., -1.),
                p(1., -1., -1.),
                p(1., -1., 1.),
                p(-1., -1., 1.),
            ); // -y
        }
        Geometry::Cylinder { radius, length } => {
            let n = 24usize;
            let h = length / 2.0;
            let ring = |k: usize| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                (radius * a.cos(), radius * a.sin())
            };
            for k in 0..n {
                let (x0, y0) = ring(k);
                let (x1, y1) = ring(k + 1);
                tris.push([
                    Point3::new(x0, y0, -h),
                    Point3::new(x1, y1, -h),
                    Point3::new(x1, y1, h),
                ]);
                tris.push([
                    Point3::new(x0, y0, -h),
                    Point3::new(x1, y1, h),
                    Point3::new(x0, y0, h),
                ]);
                tris.push([
                    Point3::new(0.0, 0.0, h),
                    Point3::new(x0, y0, h),
                    Point3::new(x1, y1, h),
                ]);
                tris.push([
                    Point3::new(0.0, 0.0, -h),
                    Point3::new(x1, y1, -h),
                    Point3::new(x0, y0, -h),
                ]);
            }
        }
        Geometry::Sphere { radius } => {
            let (lat, lon) = (12usize, 24usize);
            let at = |i: usize, j: usize| {
                let theta = std::f64::consts::PI * i as f64 / lat as f64;
                let phi = std::f64::consts::TAU * j as f64 / lon as f64;
                Point3::new(
                    radius * theta.sin() * phi.cos(),
                    radius * theta.sin() * phi.sin(),
                    radius * theta.cos(),
                )
            };
            for i in 0..lat {
                for j in 0..lon {
                    let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
                    if i > 0 {
                        tris.push([a, b, d]);
                    }
                    if i + 1 < lat {
                        tris.push([b, c, d]);
                    }
                }
            }
        }
        Geometry::Mesh { path, scale } => return mesh_tris(path, scale, cell),
    }
    Some(Arc::new(tris))
}

/// A shape's triangles in the owner's frame (the shape origin applied).
fn shape_tris(shape: &Shape, cell: Option<f64>) -> Option<Arc<Vec<[Point3<f64>; 3]>>> {
    let local = tessellate(&shape.geometry, cell)?;
    if shape.origin == Isometry3::identity() {
        return Some(local);
    }
    Some(Arc::new(
        local
            .iter()
            .map(|t| {
                [
                    shape.origin * t[0],
                    shape.origin * t[1],
                    shape.origin * t[2],
                ]
            })
            .collect(),
    ))
}

impl RenderScene {
    /// Tessellates every body of `scene` that the picture may show;
    /// meshes are decimated to `decimate` metres a cell when given.
    pub fn build(scene: &Scene, geometry: RenderGeometry, decimate: Option<f64>) -> Self {
        let mut bodies = Vec::new();
        for (i, o) in scene.obstacles().iter().enumerate() {
            if let Some(tris) = tessellate(&o.geometry, decimate) {
                if !tris.is_empty() {
                    bodies.push(Body {
                        owner: Owner::Obstacle(i),
                        id: 1 + i as i32,
                        tris,
                        color: None,
                    });
                }
            }
        }
        let mut next = 1 + scene.obstacles().len() as i32;
        for (r, robot) in scene.robots().iter().enumerate() {
            for (l, link) in robot.model.links.iter().enumerate() {
                let shapes = match geometry {
                    RenderGeometry::Visual => &link.visuals,
                    RenderGeometry::Collision => &link.collisions,
                };
                for shape in shapes {
                    if let Some(tris) = shape_tris(shape, decimate) {
                        if !tris.is_empty() {
                            bodies.push(Body {
                                owner: Owner::Link { robot: r, link: l },
                                id: next,
                                tris,
                                color: shape.color,
                            });
                        }
                    }
                }
                next += 1;
            }
        }
        RenderScene { geometry, bodies }
    }

    /// Triangles held, for tests and sizing.
    pub fn triangle_count(&self) -> usize {
        self.bodies.iter().map(|b| b.tris.len()).sum()
    }

    /// The picture `camera` takes from `pose` (its world frame) of the
    /// world `view` stands in, `width × height` pixels — depth and ids,
    /// and a flat-shaded colour image when a `lighting` is given.
    pub fn render(
        &self,
        view: &WorldView<'_>,
        camera: &Camera,
        pose: &Isometry3<f64>,
        width: usize,
        height: usize,
        lighting: Option<&Lighting>,
    ) -> Frame {
        let scene = view.scene();
        let (fx, fy, cx, cy) = intrinsics(camera.fov_deg, width, height);
        let (near, far) = (camera.near.max(1e-6), camera.far);
        let world_to_cam = pose.inverse();
        let link_poses: Vec<Vec<Isometry3<f64>>> = (0..scene.robots().len())
            .map(|r| view.link_poses(r).unwrap_or_default())
            .collect();
        let mut depth = vec![0.0f32; width * height];
        let mut id = vec![0i32; width * height];
        let mut zbuf = vec![f64::INFINITY; width * height];
        // With a light: the unshaded colour and the Lambert term per
        // pixel, composed at the end (after the shadow test).
        let mut albedo_buf = lighting.map(|_| vec![[0u8; 3]; width * height]);
        let mut lambert_buf = lighting.map(|_| vec![0.0f32; width * height]);
        let mut poly: Vec<Point3<f64>> = Vec::with_capacity(4);
        // Every shown body's triangles in the world, for the shadow map.
        let mut shadow_casters: Vec<[Point3<f64>; 3]> = Vec::new();
        let cast = lighting.is_some_and(|l| l.shadows);
        for body in &self.bodies {
            let (body_pose, albedo) = match body.owner {
                Owner::Obstacle(i) => {
                    let o = &scene.obstacles()[i];
                    let shown = match self.geometry {
                        RenderGeometry::Visual => o.visible,
                        RenderGeometry::Collision => o.enabled,
                    };
                    if !shown {
                        continue;
                    }
                    (o.pose, o.color.unwrap_or(OBSTACLE_ALBEDO))
                }
                Owner::Link { robot, link } => {
                    match link_poses.get(robot).and_then(|p| p.get(link)) {
                        Some(p) => (*p, body.color.unwrap_or(LINK_ALBEDO)),
                        None => continue,
                    }
                }
            };
            let albedo8 = [
                (albedo[0].clamp(0.0, 1.0) * 255.0).round() as u8,
                (albedo[1].clamp(0.0, 1.0) * 255.0).round() as u8,
                (albedo[2].clamp(0.0, 1.0) * 255.0).round() as u8,
            ];
            let to_cam = world_to_cam * body_pose;
            for tri in body.tris.iter() {
                let world = [body_pose * tri[0], body_pose * tri[1], body_pose * tri[2]];
                if cast {
                    shadow_casters.push(world);
                }
                let c = [to_cam * tri[0], to_cam * tri[1], to_cam * tri[2]];
                // Behind the near plane entirely: nothing to draw.
                if c.iter().all(|p| p.z > -near) {
                    continue;
                }
                let shading = lighting.map(|light| {
                    // Flat Lambert on the world-space face normal,
                    // two-sided so mesh winding never blackens a face.
                    let n = (world[1] - world[0]).cross(&(world[2] - world[0]));
                    let lambert = if n.norm() > 1e-18 {
                        n.normalize().dot(&light.direction).abs()
                    } else {
                        0.0
                    };
                    (albedo8, lambert as f32)
                });
                poly.clear();
                clip_near(&c, near, &mut poly);
                for k in 1..poly.len().saturating_sub(1) {
                    raster_tri(
                        [poly[0], poly[k], poly[k + 1]],
                        (fx, fy, cx, cy),
                        far,
                        width,
                        height,
                        (body.id, shading),
                        &mut zbuf,
                        &mut depth,
                        &mut id,
                        (albedo_buf.as_deref_mut(), lambert_buf.as_deref_mut()),
                    );
                }
            }
        }
        let rgb = match (lighting, albedo_buf, lambert_buf) {
            (Some(light), Some(albedo), Some(lambert)) => {
                // The world point a pixel shows, from its depth.
                let world_point = |u: usize, v: usize, d: f64| -> Point3<f64> {
                    pose * Point3::new(
                        (u as f64 + 0.5 - cx) / fx * d,
                        (cy - (v as f64 + 0.5)) / fy * d,
                        -d,
                    )
                };
                let shadow = if light.shadows {
                    // The map is fitted to what the picture shows (the
                    // receivers), not to the whole cell, so a wrist
                    // camera's table patch gets the map's full
                    // resolution however far the walls are.
                    let receivers = (0..height)
                        .flat_map(|v| (0..width).map(move |u| (u, v)))
                        .filter_map(|(u, v)| {
                            let d = depth[v * width + u] as f64;
                            (d > 0.0).then(|| world_point(u, v, d))
                        });
                    let size = match light.shadow_map {
                        0 => (2 * width.max(height)).clamp(64, 512),
                        n => n.max(8),
                    };
                    Some(ShadowMap::build(&shadow_casters, receivers, light, size))
                } else {
                    None
                };
                let mut out = vec![0u8; width * height * 3];
                for v in 0..height {
                    for u in 0..width {
                        let at = v * width + u;
                        let d = depth[at] as f64;
                        if d <= 0.0 {
                            continue;
                        }
                        let mut lit = lambert[at] as f64;
                        if let Some(map) = &shadow {
                            if map.shadowed(&world_point(u, v, d)) {
                                lit = 0.0;
                            }
                        }
                        let shade = light.ambient + (1.0 - light.ambient) * lit;
                        for k in 0..3 {
                            out[3 * at + k] =
                                ((albedo[at][k] as f64 * shade).clamp(0.0, 255.0)).round() as u8;
                        }
                    }
                }
                Some(out)
            }
            _ => None,
        };
        Frame {
            width,
            height,
            depth,
            id,
            rgb,
        }
    }
}

/// An orthographic depth map of the world seen from the light, fitted
/// to the points a picture shows: what a pixel is compared against to
/// know whether something stands between it and the light.
struct ShadowMap {
    u: Vector3<f64>,
    v: Vector3<f64>,
    w: Vector3<f64>,
    min_s: f64,
    min_t: f64,
    texel: f64,
    size: usize,
    /// Distance along the light ray of the nearest caster per texel.
    depth: Vec<f64>,
    bias: f64,
}

impl ShadowMap {
    /// Fits the map to `receivers` (the world points the picture shows)
    /// and rasterises every `caster` into it.
    fn build(
        casters: &[[Point3<f64>; 3]],
        receivers: impl Iterator<Item = Point3<f64>>,
        light: &Lighting,
        size: usize,
    ) -> ShadowMap {
        let w = light.direction;
        let up = if w.z.abs() < 0.9 {
            Vector3::z()
        } else {
            Vector3::x()
        };
        let u = w.cross(&up).normalize();
        let v = w.cross(&u);
        let (mut min_s, mut max_s, mut min_t, mut max_t) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for p in receivers {
            let (s, t) = (p.coords.dot(&u), p.coords.dot(&v));
            min_s = min_s.min(s);
            max_s = max_s.max(s);
            min_t = min_t.min(t);
            max_t = max_t.max(t);
        }
        if !min_s.is_finite() {
            (min_s, max_s, min_t, max_t) = (0.0, 1.0, 0.0, 1.0);
        }
        // Square texels over the larger side, a texel of margin around.
        let extent = (max_s - min_s).max(max_t - min_t).max(1e-6);
        let texel = extent / (size as f64 - 2.0);
        let mut map = ShadowMap {
            u,
            v,
            w,
            min_s: min_s - texel,
            min_t: min_t - texel,
            texel,
            size,
            depth: vec![f64::INFINITY; size * size],
            bias: 1.5 * texel + 0.005,
        };
        for tri in casters {
            map.splat(tri);
        }
        map
    }

    /// Light-space `(column, row, distance along the ray)` of a point.
    fn project(&self, p: &Point3<f64>) -> (f64, f64, f64) {
        (
            (p.coords.dot(&self.u) - self.min_s) / self.texel,
            (p.coords.dot(&self.v) - self.min_t) / self.texel,
            -p.coords.dot(&self.w),
        )
    }

    /// Rasterises one world triangle into the map (nearest distance).
    fn splat(&mut self, tri: &[Point3<f64>; 3]) {
        let s: Vec<(f64, f64, f64)> = tri.iter().map(|p| self.project(p)).collect();
        let area = (s[1].0 - s[0].0) * (s[2].1 - s[0].1) - (s[2].0 - s[0].0) * (s[1].1 - s[0].1);
        if area.abs() < 1e-12 {
            return;
        }
        let inv = 1.0 / area;
        let last = self.size as f64 - 1.0;
        let min_x = s
            .iter()
            .map(|p| p.0)
            .fold(f64::INFINITY, f64::min)
            .floor()
            .max(0.0) as isize;
        let max_x = s
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil()
            .min(last) as isize;
        let min_y = s
            .iter()
            .map(|p| p.1)
            .fold(f64::INFINITY, f64::min)
            .floor()
            .max(0.0) as isize;
        let max_y = s
            .iter()
            .map(|p| p.1)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil()
            .min(last) as isize;
        for py in min_y..=max_y {
            let yc = py as f64 + 0.5;
            for px in min_x..=max_x {
                let xc = px as f64 + 0.5;
                let w0 = ((s[1].0 - xc) * (s[2].1 - yc) - (s[2].0 - xc) * (s[1].1 - yc)) * inv;
                let w1 = ((s[2].0 - xc) * (s[0].1 - yc) - (s[0].0 - xc) * (s[2].1 - yc)) * inv;
                let w2 = 1.0 - w0 - w1;
                if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                    continue;
                }
                let d = w0 * s[0].2 + w1 * s[1].2 + w2 * s[2].2;
                let at = py as usize * self.size + px as usize;
                if d < self.depth[at] {
                    self.depth[at] = d;
                }
            }
        }
    }

    /// Whether something nearer the light covers `p`.
    fn shadowed(&self, p: &Point3<f64>) -> bool {
        let (x, y, d) = self.project(p);
        let (col, row) = (x.floor(), y.floor());
        if col < 0.0 || row < 0.0 || col >= self.size as f64 || row >= self.size as f64 {
            return false;
        }
        let nearest = self.depth[row as usize * self.size + col as usize];
        d > nearest + self.bias
    }
}

/// Sutherland–Hodgman against the plane `z = -near` (camera space,
/// `-Z` forward): the part of the triangle in front of the near plane,
/// as a polygon of 3 or 4 vertices.
fn clip_near(tri: &[Point3<f64>; 3], near: f64, out: &mut Vec<Point3<f64>>) {
    let inside = |p: &Point3<f64>| p.z <= -near;
    for i in 0..3 {
        let a = tri[i];
        let b = tri[(i + 1) % 3];
        let (ia, ib) = (inside(&a), inside(&b));
        if ia {
            out.push(a);
        }
        if ia != ib {
            let t = (-near - a.z) / (b.z - a.z);
            out.push(a + (b - a) * t);
        }
    }
}

/// Rasterises one camera-space triangle (all vertices in front of the
/// near plane) with a perspective-correct z-buffer.
#[allow(clippy::too_many_arguments)]
fn raster_tri(
    tri: [Point3<f64>; 3],
    (fx, fy, cx, cy): (f64, f64, f64, f64),
    far: f64,
    width: usize,
    height: usize,
    (body_id, shading): (i32, Option<([u8; 3], f32)>),
    zbuf: &mut [f64],
    depth: &mut [f32],
    id: &mut [i32],
    (mut albedo, mut lambert): (Option<&mut [[u8; 3]]>, Option<&mut [f32]>),
) {
    // Screen coordinates (pixels; +v down) and inverse depth.
    let mut s = [[0.0f64; 3]; 3];
    for (k, p) in tri.iter().enumerate() {
        let z = -p.z; // > 0 in front
        s[k] = [cx + fx * p.x / z, cy - fy * p.y / z, 1.0 / z];
    }
    let area =
        (s[1][0] - s[0][0]) * (s[2][1] - s[0][1]) - (s[2][0] - s[0][0]) * (s[1][1] - s[0][1]);
    if area.abs() < 1e-12 {
        return;
    }
    let min_x = s
        .iter()
        .map(|v| v[0])
        .fold(f64::INFINITY, f64::min)
        .floor()
        .max(0.0) as isize;
    let max_x = s
        .iter()
        .map(|v| v[0])
        .fold(f64::NEG_INFINITY, f64::max)
        .ceil()
        .min(width as f64 - 1.0) as isize;
    let min_y = s
        .iter()
        .map(|v| v[1])
        .fold(f64::INFINITY, f64::min)
        .floor()
        .max(0.0) as isize;
    let max_y = s
        .iter()
        .map(|v| v[1])
        .fold(f64::NEG_INFINITY, f64::max)
        .ceil()
        .min(height as f64 - 1.0) as isize;
    if min_x > max_x || min_y > max_y {
        return;
    }
    let inv = 1.0 / area;
    for py in min_y..=max_y {
        let yc = py as f64 + 0.5;
        for px in min_x..=max_x {
            let xc = px as f64 + 0.5;
            // Barycentric weights from edge functions.
            let w0 = ((s[1][0] - xc) * (s[2][1] - yc) - (s[2][0] - xc) * (s[1][1] - yc)) * inv;
            let w1 = ((s[2][0] - xc) * (s[0][1] - yc) - (s[0][0] - xc) * (s[2][1] - yc)) * inv;
            let w2 = 1.0 - w0 - w1;
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                continue;
            }
            let inv_z = w0 * s[0][2] + w1 * s[1][2] + w2 * s[2][2];
            if inv_z <= 0.0 {
                continue;
            }
            let z = 1.0 / inv_z;
            if z > far {
                continue;
            }
            let at = py as usize * width + px as usize;
            if z < zbuf[at] {
                zbuf[at] = z;
                depth[at] = z as f32;
                id[at] = body_id;
                if let Some((color, l)) = shading {
                    if let Some(buf) = albedo.as_deref_mut() {
                        buf[at] = color;
                    }
                    if let Some(buf) = lambert.as_deref_mut() {
                        buf[at] = l;
                    }
                }
            }
        }
    }
}

/// Camera-frame points of a depth image (`x` right, `y` up, `z` toward
/// the viewer — the camera convention), `(0, 0, 0)` for a pixel with no
/// return. Row-major, row 0 the top.
pub fn points(frame: &Frame, fov_deg: f64) -> Vec<[f64; 3]> {
    let (fx, fy, cx, cy) = intrinsics(fov_deg, frame.width, frame.height);
    let mut out = Vec::with_capacity(frame.width * frame.height);
    for v in 0..frame.height {
        for u in 0..frame.width {
            let d = frame.depth[v * frame.width + u] as f64;
            if d <= 0.0 {
                out.push([0.0, 0.0, 0.0]);
            } else {
                out.push([
                    (u as f64 + 0.5 - cx) / fx * d,
                    (cy - (v as f64 + 0.5)) / fy * d,
                    -d,
                ]);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::{CameraMount, Condition, Sequence, Step};
    use nalgebra::{UnitQuaternion, Vector3};

    fn box_cell() -> Scene {
        let mut scene = Scene::empty();
        // A 1 m cube, its near face at x = 2 (the camera looks down +x).
        scene
            .add_obstacle(
                "cube",
                Geometry::Box {
                    size: Vector3::new(1.0, 1.0, 1.0),
                },
                Isometry3::translation(2.5, 0.0, 0.0),
            )
            .unwrap();
        scene
            .add_obstacle(
                "far_wall",
                Geometry::Box {
                    size: Vector3::new(0.2, 6.0, 6.0),
                },
                Isometry3::translation(20.0, 0.0, 0.0),
            )
            .unwrap();
        // An upright camera looking down +x: camera -Z → world +x, camera
        // +Y (image up) → world +z, hence camera +X → world -y. Stated as
        // the basis, not a hand-derived angle (a turn about Y alone would
        // leave the picture lying on its side).
        let rotation = UnitQuaternion::from_rotation_matrix(
            &nalgebra::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::from_columns(&[
                Vector3::new(0.0, -1.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
                Vector3::new(-1.0, 0.0, 0.0),
            ])),
        );
        scene
            .upsert_camera(Camera {
                name: "cam".into(),
                mount: CameraMount::World,
                pose: Isometry3::from_parts(nalgebra::Translation3::new(0.0, 0.0, 0.0), rotation),
                fov_deg: 60.0,
                resolution: [64, 48],
                near: 0.1,
                far: 10.0,
            })
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "hold".into(),
            steps: vec![Step {
                name: "wait".into(),
                actions: vec![],
                transition: Condition::Elapsed { seconds: 1.0 },
                select: Vec::new(),
            }],
        });
        scene
    }

    #[test]
    fn a_cube_in_front_of_the_camera_reads_its_face_distance() {
        let scene = box_cell();
        let live = scene
            .open_rollout(&["hold"], &crate::rollout::RolloutOptions::default(), None)
            .unwrap();
        let view = live.view();
        let frame = view.render(0, RenderGeometry::Visual, 64, 48).unwrap();
        assert!(frame.rgb.is_none());
        let center = frame.depth[24 * 64 + 32];
        assert!((center - 2.0).abs() < 1e-4, "center depth {center}");
        assert_eq!(frame.id[24 * 64 + 32], 1);
        // The cube spans ±0.5 m at 2 m: half the 60° field (±0.577 at
        // 1 m → ±1.15 m at 2 m) — so about 43% of the width is cube,
        // the rest background (the far wall is beyond `far`).
        let row: Vec<f32> = frame.depth[24 * 64..25 * 64].to_vec();
        let cube_px = row.iter().filter(|d| (**d - 2.0).abs() < 1e-3).count();
        assert!((26..=30).contains(&cube_px), "cube pixels {cube_px}");
        assert_eq!(row[0], 0.0);
        assert_eq!(frame.id[24 * 64], 0);
        // Same world, same picture, bit for bit.
        let again = view.render(0, RenderGeometry::Visual, 64, 48).unwrap();
        assert_eq!(frame, again);
        // Points: the centre pixel is 2 m straight ahead (-z), a right-hand
        // cube pixel has +x, an upper one +y.
        let pts = points(&frame, 60.0);
        let c = pts[24 * 64 + 32];
        assert!(
            c[0].abs() < 0.05 && c[1].abs() < 0.05 && (c[2] + 2.0).abs() < 1e-4,
            "{c:?}"
        );
        let right = pts[24 * 64 + 40];
        assert!(right[0] > 0.1, "{right:?}");
        let up = pts[16 * 64 + 32];
        assert!(up[1] > 0.1, "{up:?}");
        assert_eq!(pts[0], [0.0, 0.0, 0.0]);
        assert_eq!(
            segmentation_ids(&scene),
            vec![("cube".to_string(), 1), ("far_wall".to_string(), 2)]
        );
        assert!(view.render(3, RenderGeometry::Visual, 8, 8).is_none());
    }

    #[test]
    fn a_shaded_picture_paints_colour_by_light() {
        let mut scene = box_cell();
        scene
            .set_obstacle_color("cube", Some([1.0, 0.2, 0.2]))
            .unwrap();
        let live = scene
            .open_rollout(&["hold"], &crate::rollout::RolloutOptions::default(), None)
            .unwrap();
        // Lit head-on (the light behind the camera, along -x onto the
        // face): full Lambert on the face the camera sees.
        let mut view_live = live;
        view_live.set_lighting(Lighting::new(Vector3::new(-1.0, 0.0, 0.0), 0.2));
        let frame = view_live
            .view()
            .render_shaded(0, RenderGeometry::Visual, 64, 48)
            .unwrap();
        let rgb = frame.rgb.as_ref().unwrap();
        let at = 3 * (24 * 64 + 32);
        assert_eq!(&rgb[at..at + 3], &[255, 51, 51], "lit red face");
        assert_eq!(&rgb[0..3], &[0, 0, 0], "background is black");
        // Lit from the side: the face only gets the ambient share.
        view_live.set_lighting(Lighting::new(Vector3::new(0.0, 1.0, 0.0), 0.2));
        let frame = view_live
            .view()
            .render_shaded(0, RenderGeometry::Visual, 64, 48)
            .unwrap();
        let rgb = frame.rgb.as_ref().unwrap();
        assert_eq!(&rgb[at..at + 3], &[51, 10, 10], "ambient only");
        // Depth and ids are the same picture whatever the light.
        let plain = view_live
            .view()
            .render(0, RenderGeometry::Visual, 64, 48)
            .unwrap();
        assert_eq!(plain.depth, frame.depth);
        assert_eq!(plain.id, frame.id);
    }

    #[test]
    fn shadows_fall_from_an_off_screen_caster() {
        let mut scene = box_cell();
        scene
            .set_obstacle_color("cube", Some([1.0, 0.2, 0.2]))
            .unwrap();
        // The far wall only widens the shadow map; leave it out.
        scene.set_obstacle_visible("far_wall", false).unwrap();
        // An awning above and in front of the cube, out of the camera's
        // 23° vertical half-field: with the light coming from up-front
        // (toward (-1, 0, 1)) it shades the top strip z ∈ [0.3, 0.5] of
        // the face the camera sees, and nothing else.
        scene
            .add_obstacle(
                "awning",
                Geometry::Box {
                    size: Vector3::new(0.2, 2.0, 0.05),
                },
                Isometry3::translation(1.5, 0.0, 0.9),
            )
            .unwrap();
        let mut live = scene
            .open_rollout(&["hold"], &crate::rollout::RolloutOptions::default(), None)
            .unwrap();
        let light = Lighting::new(Vector3::new(-1.0, 0.0, 1.0), 0.2);
        live.set_lighting(light.clone());
        let plain = live
            .view()
            .render_shaded(0, RenderGeometry::Visual, 64, 48)
            .unwrap();
        live.set_lighting(light.with_shadows(true));
        let shadowed = live
            .view()
            .render_shaded(0, RenderGeometry::Visual, 64, 48)
            .unwrap();
        let px = |f: &Frame, row: usize, col: usize| -> [u8; 3] {
            let rgb = f.rgb.as_ref().unwrap();
            let at = 3 * (row * 64 + col);
            [rgb[at], rgb[at + 1], rgb[at + 2]]
        };
        // Lambert cos 45° on the face: 0.2 + 0.8·0.707 of the albedo.
        assert_eq!(px(&plain, 24, 32), [195, 39, 39]);
        assert_eq!(
            px(&plain, 13, 32),
            [195, 39, 39],
            "no shadow without the map"
        );
        assert_eq!(px(&shadowed, 24, 32), [195, 39, 39], "the centre stays lit");
        // Row 13 looks at z ≈ 0.38 on the face: under the awning.
        assert_eq!(
            px(&shadowed, 13, 32),
            [51, 10, 10],
            "ambient only in the shadow"
        );
        // A pixel is only ever shadowed by something nearer the light,
        // never by its own face: the lit rows are identical in both.
        for row in 20..40 {
            assert_eq!(px(&plain, row, 32), px(&shadowed, row, 32), "row {row}");
        }
        assert_eq!(plain.depth, shadowed.depth);
        assert_eq!(plain.id, shadowed.id);
    }

    #[test]
    fn decimation_cuts_a_dense_mesh_and_leaves_primitives() {
        // A 1 m plane meshed 20 × 20 (800 triangles), as an OBJ file.
        let dir =
            std::env::temp_dir().join(format!("botrail-raster-decimate-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("plane.obj");
        let n = 20usize;
        let mut obj = String::new();
        for j in 0..=n {
            for i in 0..=n {
                obj.push_str(&format!(
                    "v {} {} 0\n",
                    i as f64 / n as f64,
                    j as f64 / n as f64
                ));
            }
        }
        for j in 0..n {
            for i in 0..n {
                let a = j * (n + 1) + i + 1;
                let (b, c, d) = (a + 1, a + n + 1, a + n + 2);
                obj.push_str(&format!("f {a} {b} {d}\nf {a} {d} {c}\n"));
            }
        }
        std::fs::write(&path, obj).unwrap();
        let full = tessellate(
            &Geometry::Mesh {
                path: path.clone(),
                scale: Vector3::new(1.0, 1.0, 1.0),
            },
            None,
        )
        .unwrap();
        assert_eq!(full.len(), 800);
        let coarse = decimate(&full, 0.5);
        assert!(
            !coarse.is_empty() && coarse.len() < 40,
            "{} triangles at a 0.5 m cell",
            coarse.len()
        );
        assert_eq!(decimate(&full, 0.0).len(), 800, "no cell, no cut");
        // Through the rollout: the picture's triangle count follows the
        // world's decimation setting, and the mesh cache keys on it.
        let mut scene = box_cell();
        scene
            .add_obstacle(
                "plane",
                Geometry::Mesh {
                    path,
                    scale: Vector3::new(1.0, 1.0, 1.0),
                },
                Isometry3::translation(2.0, -0.5, -0.5),
            )
            .unwrap();
        let mut live = scene
            .open_rollout(&["hold"], &crate::rollout::RolloutOptions::default(), None)
            .unwrap();
        assert_eq!(live.view().render_triangles(RenderGeometry::Visual), None);
        live.view()
            .render(0, RenderGeometry::Visual, 16, 12)
            .unwrap();
        let all = live
            .view()
            .render_triangles(RenderGeometry::Visual)
            .unwrap();
        assert_eq!(all, 800 + 2 * 12, "the plane and two boxes");
        live.set_render_decimate(Some(0.5));
        live.view()
            .render(0, RenderGeometry::Visual, 16, 12)
            .unwrap();
        let cut = live
            .view()
            .render_triangles(RenderGeometry::Visual)
            .unwrap();
        assert_eq!(cut, coarse.len() + 2 * 12, "boxes are not decimated");
        live.set_render_decimate(None);
        live.view()
            .render(0, RenderGeometry::Visual, 16, 12)
            .unwrap();
        assert_eq!(
            live.view().render_triangles(RenderGeometry::Visual),
            Some(all)
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn near_and_far_clip_and_visibility_follow_the_picture() {
        let mut scene = box_cell();
        // A slab around the camera, x ∈ [-0.1, 0.45]: its front face is
        // behind the camera (clipped away, not projected into garbage),
        // its side faces cross the near plane and draw from there, and
        // the centre pixel sees the inside of its back face.
        scene
            .add_obstacle(
                "slab",
                Geometry::Box {
                    size: Vector3::new(0.55, 0.2, 0.2),
                },
                Isometry3::translation(0.175, 0.0, 0.0),
            )
            .unwrap();
        let live = scene
            .open_rollout(&["hold"], &crate::rollout::RolloutOptions::default(), None)
            .unwrap();
        let frame = live
            .view()
            .render(0, RenderGeometry::Visual, 64, 48)
            .unwrap();
        let center = frame.depth[24 * 64 + 32];
        assert!(
            (center - 0.45).abs() < 1e-4,
            "back face from inside: {center}"
        );
        assert_eq!(frame.id[24 * 64 + 32], 3);
        // The top row looks up at the slab's ceiling (z = 0.1): with
        // fx = fy = 32 / tan 30° the row-0 ray climbs 23.5 / fx per metre,
        // meeting the ceiling 0.236 m out — inside the near-clipped face.
        let top = frame.depth[32];
        assert!(
            (top - 0.236).abs() < 0.01,
            "ceiling through the near plane: {top}"
        );
        assert_eq!(frame.id[32], 3);
        // Hidden obstacles vanish from the visual picture but stay in the
        // collision one; disabled ones the other way round.
        let mut scene = box_cell();
        scene.set_obstacle_visible("cube", false).unwrap();
        let live = scene
            .open_rollout(&["hold"], &crate::rollout::RolloutOptions::default(), None)
            .unwrap();
        assert_eq!(
            live.view()
                .render(0, RenderGeometry::Visual, 64, 48)
                .unwrap()
                .depth[24 * 64 + 32],
            0.0
        );
        assert!(
            (live
                .view()
                .render(0, RenderGeometry::Collision, 64, 48)
                .unwrap()
                .depth[24 * 64 + 32]
                - 2.0)
                .abs()
                < 1e-4
        );
        let mut scene = box_cell();
        scene.set_obstacle_enabled("cube", false).unwrap();
        let live = scene
            .open_rollout(&["hold"], &crate::rollout::RolloutOptions::default(), None)
            .unwrap();
        assert_eq!(
            live.view()
                .render(0, RenderGeometry::Collision, 64, 48)
                .unwrap()
                .depth[24 * 64 + 32],
            0.0
        );
        assert!(
            (live
                .view()
                .render(0, RenderGeometry::Visual, 64, 48)
                .unwrap()
                .depth[24 * 64 + 32]
                - 2.0)
                .abs()
                < 1e-4
        );
    }
}
