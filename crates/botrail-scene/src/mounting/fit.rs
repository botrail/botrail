//! Bounded drawing review of adjacent, planar bolted interfaces.
//! No nominalisation of ranges, inferred tolerances or mesh measurements.

use super::*;
use nalgebra::{Isometry3, Point3, Vector3};

mod fasteners;
mod geometry;

// Numerical roundoff only, in millimetres (not a manufacturing tolerance).
const EPS: f64 = 1e-7;

#[derive(Debug, Clone, Serialize)]
struct Check {
    check: String,
    status: &'static str,
    message: String,
    values: Value,
}

#[derive(Default)]
struct Checks(Vec<Check>);

impl Checks {
    fn add(
        &mut self,
        key: impl Into<String>,
        status: &'static str,
        message: impl Into<String>,
        values: Value,
    ) {
        self.0.push(Check {
            check: key.into(),
            status,
            message: message.into(),
            values,
        });
    }

    fn unknown(&mut self, key: &str, message: &str) {
        self.add(key, "unknown", message, Value::Null);
    }

    fn status(&self) -> &'static str {
        if self.0.iter().any(|c| c.status == "fail") {
            "fail"
        } else if self.0.is_empty() || self.0.iter().any(|c| c.status == "unknown") {
            "unknown"
        } else {
            "pass"
        }
    }
}

#[derive(Clone, Copy)]
struct Face<'a> {
    part: &'a Part,
    face: &'a MountInterface,
}

impl Face<'_> {
    fn geometry(&self) -> Option<&MountGeometry> {
        self.face
            .geometry
            .as_ref()
            .filter(|g| g.frame_verified && self.part.supported(&g.evidence))
    }
    fn clearance(&self) -> Option<&MountClearance> {
        self.face
            .clearance
            .as_ref()
            .filter(|c| c.frame_verified && self.part.supported(&c.evidence))
    }
}

/// All realised values satisfy the upper bound; partial overlap is unknown.
fn below(actual: DimensionRange, limit: DimensionRange) -> &'static str {
    if actual.max <= limit.min + EPS {
        "pass"
    } else if actual.min > limit.max + EPS {
        "fail"
    } else {
        "unknown"
    }
}

fn contained(actual: DimensionRange, allowed: DimensionRange) -> &'static str {
    if actual.min >= allowed.min - EPS && actual.max <= allowed.max + EPS {
        "pass"
    } else if actual.max < allowed.min - EPS || actual.min > allowed.max + EPS {
        "fail"
    } else {
        "unknown"
    }
}

fn point(pose: &Isometry3<f64>, xy: [f64; 2]) -> Vector3<f64> {
    pose.transform_point(&Point3::new(xy[0], xy[1], 0.0)).coords
}

pub(super) fn review(edge: &Edge, graph: &Graph<'_>, report: &mut MountingReport) {
    let a = edge.base.and_then(|i| {
        let part = &graph.parts[i];
        Some(Face {
            part,
            face: part.face(&edge.flange, InterfaceRole::Flange)?,
        })
    });
    let b = edge.tool.and_then(|i| {
        let part = &graph.parts[i];
        Some(Face {
            part,
            face: part.face(&edge.mount, InterfaceRole::Mount)?,
        })
    });
    let documents = |face: Option<Face<'_>>| {
        face.map(|f| json!({
        "target": f.part.name, "catalog": f.part.identity, "interface": f.face, "sources": f.part.meta.sources
    }))
    };
    let evidence = json!({"flange": documents(a), "mount": documents(b), "offset": edge.offset});
    let mut add = |key: &str, checks: Checks| {
        let status = checks.status();
        let issues: Vec<_> = checks
            .0
            .iter()
            .filter(|c| c.status != "pass")
            .map(|c| c.message.as_str())
            .collect();
        let message = if status == "pass" {
            format!("{key}: all declared checks satisfy their bounds")
        } else {
            format!("{key}: {}", issues.join("; "))
        };
        report.add(&edge.target, key, status, message,
            if status == "pass" { "" } else { "Review the listed drawing data and the actual adjacent parts; supply missing evidence or correct the assembly" },
            json!({"scope": "adjacent_planar_bolted_interfaces", "assembly": evidence, "checks": checks.0}));
    };
    let mut required = Checks::default();
    match b {
        Some(f) if f.face.requirements_complete && f.part.supported(&f.face.evidence) => required
            .add(
                "coverage",
                "pass",
                "The installation declaration enumerates required separate parts",
                Value::Null,
            ),
        _ => required.unknown(
            "coverage",
            "The complete list of required separate parts is not documented",
        ),
    }
    add("requirements_complete", required);

    let mut interface = Checks::default();
    match a
        .zip(b)
        .filter(|(a, b)| a.part.supported(&a.face.evidence) && b.part.supported(&b.face.evidence))
        .and_then(|(a, b)| {
            a.face
                .interface_id
                .as_ref()
                .zip(b.face.interface_id.as_ref())
        }) {
        Some((a, b)) => interface.add(
            "bare_interface",
            if a == b { "pass" } else { "fail" },
            format!("Declared bare interfaces `{a}` and `{b}` compared; this is not host approval"),
            json!({"flange_interface": a, "mount_interface": b}),
        ),
        None => interface.unknown(
            "bare_interface",
            "Both bare mating interfaces need identifiers and supporting evidence",
        ),
    }
    add("interface", interface);

    let mut pose_checks = Checks::default();
    match b
        .filter(|f| f.part.supported(&f.face.evidence))
        .and_then(|f| f.face.allowed_poses.as_ref())
    {
        Some(poses) => pose_checks.add(
            "allowed_poses",
            if poses.iter().any(|p| pose_matches(p, &edge.offset)) {
                "pass"
            } else {
                "fail"
            },
            "Recorded flange-to-mount pose compared with explicitly permitted poses",
            json!(poses),
        ),
        None => pose_checks.unknown(
            "allowed_poses",
            "No evidenced permitted mounting pose is declared",
        ),
    }
    add("declared_pose", pose_checks);

    let mut g = Checks::default();
    let mut screws = Checks::default();
    let mut space = Checks::default();
    if let Some((a, b)) = a.zip(b).filter(|_| {
        edge.offset
            .position
            .iter()
            .chain(&edge.offset.quaternion)
            .all(|v| v.is_finite())
    }) {
        let mut pose = Isometry3::from(&edge.offset);
        pose.translation.vector *= 1000.0;
        if let Some((ga, gb)) = a.geometry().zip(b.geometry()) {
            let pairs = geometry::review(ga, gb, &pose, &mut g);
            fasteners::review(a, b, ga, gb, pairs.as_deref(), &mut screws);
        } else {
            g.unknown(
                "evidence",
                "Both mating geometries need source evidence and verified drawing frames",
            );
            screws.unknown(
                "evidence",
                "Fastening cannot be matched without both verified mating geometries",
            );
        }
        clearance(a, b, &pose, &mut space);
    } else {
        g.unknown(
            "faces",
            "Both actual mating faces and a finite assembly pose are required",
        );
        screws.unknown(
            "faces",
            "Both actual mating faces and their fastener declarations are required",
        );
        space.unknown(
            "faces",
            "Both actual mating faces and their clearance declarations are required",
        );
    }
    add("geometry", g);
    add("fasteners", screws);
    add("clearance", space);
}

fn clearance(a: Face<'_>, b: Face<'_>, pose: &Isometry3<f64>, checks: &mut Checks) {
    let Some((a, b)) = a.clearance().zip(b.clearance()) else {
        checks.unknown(
            "evidence",
            "Both clearance declarations need source evidence and verified drawing frames",
        );
        return;
    };
    if !a.complete || !b.complete {
        checks.unknown(
            "coverage",
            "Conservative body and access envelopes are incomplete",
        );
    }
    for (kind, left, right) in [
        ("body", &a.solids, &b.solids),
        ("flange_access", &a.access, &b.solids),
        ("mount_access", &a.solids, &b.access),
    ] {
        for x in left {
            for y in right {
                let px = Isometry3::translation(x.center_mm[0], x.center_mm[1], x.center_mm[2]);
                let py =
                    pose * Isometry3::translation(y.center_mm[0], y.center_mm[1], y.center_mm[2]);
                let sx = parry3d_f64::shape::Cuboid::new(
                    parry3d_f64::math::Vector::from(x.size_mm) / 2.0,
                );
                let sy = parry3d_f64::shape::Cuboid::new(
                    parry3d_f64::math::Vector::from(y.size_mm) / 2.0,
                );
                let result = parry3d_f64::query::contact(
                    &botrail_collide::to_parry_pose(&px),
                    &sx,
                    &botrail_collide::to_parry_pose(&py),
                    &sy,
                    0.0,
                );
                let status = match result {
                    Ok(Some(contact)) if contact.dist < -EPS => "fail",
                    Ok(_) => "pass",
                    Err(_) => "unknown",
                };
                checks.add(
                    format!("{kind}:{}:{}", x.id, y.id),
                    status,
                    format!(
                        "{kind} envelopes `{}` and `{}` {}",
                        x.id,
                        y.id,
                        if status == "fail" {
                            "overlap"
                        } else {
                            "checked in assembly pose"
                        }
                    ),
                    json!({"flange_box": x, "mount_box": y}),
                );
            }
        }
    }
    // complete requires solids on each side, so at least a body pair exists.
}
