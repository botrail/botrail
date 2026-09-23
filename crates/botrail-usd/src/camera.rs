//! `UsdGeomCamera` → botrail camera optics, shared by the scene importer
//! (a `Camera` prim in a layout becomes a world fixture) and the robot
//! importer (one under a rigid body becomes a link camera).
//!
//! USD and botrail agree on the camera frame (-Z is the view direction, +Y
//! image-up), so only the optics need translating: the film back
//! (`focalLength` / `horizontalAperture`, nominal millimeters — only their
//! ratio matters) becomes a horizontal field of view, the aperture aspect
//! becomes the image aspect, and `clippingRange` (stage units) becomes
//! near/far in meters. USD cameras carry no pixel count; a stage botrail
//! exported says it in `botrail:resolution`, anything else gets
//! [`DEFAULT_WIDTH`] and the aperture aspect.

use openusd::schemas::geom;
use openusd::usd::{Prim, Stage};
use openusd::{gf, tf};

/// The pinhole optics a `Camera` prim authors, in botrail's terms.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraOptics {
    /// Horizontal field of view, degrees.
    pub fov_deg: f64,
    /// Image size in pixels.
    pub resolution: [u32; 2],
    /// Near clip distance, meters.
    pub near: f64,
    /// Far clip distance, meters.
    pub far: f64,
}

/// Image width when the stage says nothing about pixels; the height
/// follows the aperture aspect.
pub const DEFAULT_WIDTH: u32 = 1280;
/// The furthest clip botrail keeps. Omniverse authors `clippingRange` fars
/// of 1e7 stage units, which would draw a frustum the size of a country and
/// flatten a depth picture to nothing.
pub const MAX_FAR: f64 = 100.0;
/// The nearest clip botrail keeps (a zero near plane has no projection).
pub const MIN_NEAR: f64 = 0.001;

/// The custom attribute botrail's exporter writes next to the film back so
/// its own stages round-trip pixel-exact (`custom int2 botrail:resolution`).
pub const RESOLUTION_ATTR: &str = "botrail:resolution";

/// Reads a `Camera` prim's optics. `mpu` scales the clipping range from
/// stage units to meters. `notes` receives what had to be adjusted or why
/// the prim was skipped (`None`: an orthographic camera, or a degenerate
/// film back or clip range), without the prim path — the caller prefixes it.
pub(crate) fn read_camera_optics(
    stage: &Stage,
    prim: &Prim,
    mpu: f64,
    notes: &mut Vec<String>,
) -> anyhow::Result<Option<CameraOptics>> {
    let Some(camera) = geom::Camera::get(stage, prim.path().clone())? else {
        return Ok(None);
    };
    if let Some(projection) = camera.projection_attr().get::<tf::Token>()? {
        if projection.as_str() == "orthographic" {
            notes.push("orthographic camera; botrail cameras are pinhole, skipped".into());
            return Ok(None);
        }
    }
    let focal = camera
        .focal_length_attr()
        .get::<f32>()?
        .map(f64::from)
        .unwrap_or(50.0);
    let h_aperture = camera
        .horizontal_aperture_attr()
        .get::<f32>()?
        .map(f64::from)
        .unwrap_or(20.955);
    let v_aperture = camera
        .vertical_aperture_attr()
        .get::<f32>()?
        .map(f64::from)
        .unwrap_or(15.2908);
    if !(focal > 0.0 && h_aperture > 0.0 && v_aperture > 0.0) {
        notes.push(format!(
            "degenerate film back (focalLength {focal}, apertures {h_aperture} x {v_aperture}); skipped"
        ));
        return Ok(None);
    }
    let fov_deg = (2.0 * (h_aperture / (2.0 * focal)).atan()).to_degrees();

    // Pixels: botrail's own stages say; everything else keeps the aspect.
    let resolution = match prim.attribute(RESOLUTION_ATTR).get::<gf::Vec2i>() {
        Ok(Some(px)) if px.x > 0 && px.y > 0 => [px.x as u32, px.y as u32],
        _ => {
            let height = (f64::from(DEFAULT_WIDTH) * v_aperture / h_aperture).round();
            [DEFAULT_WIDTH, (height as u32).max(1)]
        }
    };

    let clip = camera
        .clipping_range_attr()
        .get::<gf::Vec2f>()?
        .map(|c| [f64::from(c.x) * mpu, f64::from(c.y) * mpu])
        .unwrap_or([mpu, 1_000_000.0 * mpu]);
    let mut near = clip[0];
    let mut far = clip[1];
    if near < MIN_NEAR {
        notes.push(format!("clippingRange near {near} m raised to {MIN_NEAR} m"));
        near = MIN_NEAR;
    }
    if far > MAX_FAR {
        notes.push(format!("clippingRange far {far} m capped to {MAX_FAR} m"));
        far = MAX_FAR;
    }
    if far <= near {
        notes.push(format!("clippingRange {clip:?} has far <= near; skipped"));
        return Ok(None);
    }
    // USD authors these as floats; keep the f32 noise out of the authored
    // numbers a studio form or a generated script shows (60.0, not
    // 60.000000665). A ten-thousandth of a degree and a micrometer are far
    // below what a pixel resolves.
    // Divide by the scale rather than multiply by its reciprocal: 50000 /
    // 1e6 is exactly the double nearest 0.05, 50000 * 1e-6 is not.
    let round = |x: f64, scale: f64| (x * scale).round() / scale;
    Ok(Some(CameraOptics {
        fov_deg: round(fov_deg, 1e4),
        resolution,
        near: round(near, 1e6),
        far: round(far, 1e6),
    }))
}
