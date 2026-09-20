use super::*;

pub(super) fn review(
    a: &MountGeometry,
    b: &MountGeometry,
    pose: &Isometry3<f64>,
    checks: &mut Checks,
) -> Option<Vec<(usize, usize)>> {
    let normal = pose.rotation * Vector3::new(0.0, 0.0, b.normal_z as f64);
    if pose.translation.z.abs() > EPS
        || (normal + Vector3::new(0.0, 0.0, a.normal_z as f64)).norm() > 1e-8
    {
        checks.add("mating_plane", "fail", "Verified mating planes are separated or their outward normals do not oppose", json!({"separation_mm": pose.translation.z, "mount_normal": [normal.x, normal.y, normal.z]}));
        return None;
    }
    checks.add(
        "mating_plane",
        "pass",
        "Verified mating planes coincide and oppose",
        Value::Null,
    );
    if !a.complete || !b.complete {
        checks.unknown(
            "coverage",
            "The drawing does not enumerate all required geometry and tolerances",
        );
    }
    let holes = pair(
        &a.holes,
        &b.holes,
        a.complete,
        b.complete,
        "hole",
        |h| &h.id,
        |h| &h.id,
        |x, y| {
            let (clearance, threaded) = match (x.kind, y.kind) {
                (HoleKind::Clearance, HoleKind::Threaded) => (x, y),
                (HoleKind::Threaded, HoleKind::Clearance) => (y, x),
                _ => return None,
            };
            let distance = (Vector3::new(x.position_mm[0], x.position_mm[1], 0.0)
                - point(pose, y.position_mm))
            .norm();
            let values = json!({"flange_hole": x.id, "mount_hole": y.id, "distance_mm": distance});
            let status = match (
                clearance.diameter_mm,
                threaded.thread.as_ref(),
                x.position_tolerance_mm,
                y.position_tolerance_mm,
            ) {
                (Some(bore), Some(thread), Some(tx), Some(ty)) => radial_fit(
                    distance,
                    tx + ty,
                    DimensionRange {
                        min: thread.diameter_mm,
                        max: thread.diameter_mm,
                    },
                    bore,
                ),
                _ => "unknown",
            };
            Some(Check { check: format!("hole:{}:{}", x.id, y.id), status,
            message: format!("Hole pair `{}` / `{}`: clearance over the full diameter and position bounds is {status}", x.id, y.id), values })
        },
        checks,
    );
    pair(
        &a.locators,
        &b.locators,
        a.complete,
        b.complete,
        "locator",
        |f| &f.id,
        |f| &f.id,
        |x, y| {
            let (male, female) = match (x.kind, y.kind) {
                (LocatorKind::Pin, LocatorKind::Hole)
                | (LocatorKind::Boss, LocatorKind::Recess) => (x, y),
                (LocatorKind::Hole, LocatorKind::Pin)
                | (LocatorKind::Recess, LocatorKind::Boss) => (y, x),
                _ => return None,
            };
            let distance = (Vector3::new(x.position_mm[0], x.position_mm[1], 0.0)
                - point(pose, y.position_mm))
            .norm();
            let radial = match (
                male.diameter_mm,
                female.diameter_mm,
                x.position_tolerance_mm,
                y.position_tolerance_mm,
            ) {
                (Some(pin), Some(bore), Some(tx), Some(ty)) => {
                    radial_fit(distance, tx + ty, pin, bore)
                }
                _ => "unknown",
            };
            let depth = male
                .depth_mm
                .zip(female.depth_mm)
                .map_or("unknown", |(pin, bore)| below(pin, bore));
            let status = if [radial, depth].contains(&"fail") {
                "fail"
            } else if [radial, depth].contains(&"unknown") {
                "unknown"
            } else {
                "pass"
            };
            Some(Check {
                check: format!("locator:{}:{}", x.id, y.id),
                status,
                message: format!(
                    "Locator pair `{}` / `{}`: radial fit {radial}, depth {depth}",
                    x.id, y.id
                ),
                values: json!({"flange_locator": x.id, "mount_locator": y.id, "distance_mm": distance, "radial_status": radial, "depth_status": depth}),
            })
        },
        checks,
    );
    Some(holes)
}

/// Clearance remaining around the larger male diameter vs possible axis
/// offsets. Only guaranteed containment across every tolerance is a pass.
fn radial_fit(
    distance: f64,
    position_uncertainty: f64,
    male: DimensionRange,
    female: DimensionRange,
) -> &'static str {
    let offsets = DimensionRange {
        min: (distance - position_uncertainty).max(0.0),
        max: distance + position_uncertainty,
    };
    let gap = DimensionRange {
        min: (female.min - male.max) / 2.0,
        max: (female.max - male.min) / 2.0,
    };
    below(offsets, gap)
}

/// Match by location and complementary kind, never by feature id/list order.
/// Ambiguous assignments stay unknown; a feature cannot be used twice.
#[allow(clippy::too_many_arguments)]
fn pair<A, B>(
    a: &[A],
    b: &[B],
    a_complete: bool,
    b_complete: bool,
    kind: &str,
    a_id: impl Fn(&A) -> &str,
    b_id: impl Fn(&B) -> &str,
    compare: impl Fn(&A, &B) -> Option<Check>,
    checks: &mut Checks,
) -> Vec<(usize, usize)> {
    let candidates: Vec<Vec<_>> = a
        .iter()
        .map(|x| b.iter().map(|y| compare(x, y)).collect())
        .collect();
    let mut pairs = Vec::new();
    if a.len() != b.len() && a_complete && b_complete {
        checks.add(
            format!("{kind}:count"),
            "fail",
            format!("Complete {kind} patterns have different feature counts"),
            json!({"flange_count": a.len(), "mount_count": b.len()}),
        );
    }
    for (i, row) in candidates.iter().enumerate() {
        let possible: Vec<_> = row
            .iter()
            .enumerate()
            .filter(|(_, c)| c.as_ref().is_some_and(|c| c.status != "fail"))
            .collect();
        if possible.is_empty() {
            let status = if b_complete && (b.is_empty() || row.iter().any(Option::is_some)) {
                "fail"
            } else {
                "unknown"
            };
            checks.add(format!("{kind}:{}", a_id(&a[i])), status,
                format!("{kind} `{}` has no established complementary feature within the declared bounds", a_id(&a[i])),
                json!({"candidates": row}));
        } else if possible.len() == 1 {
            let j = possible[0].0;
            let owners = candidates
                .iter()
                .filter(|r| r[j].as_ref().is_some_and(|c| c.status != "fail"))
                .count();
            if owners == 1 {
                checks.0.push(row[j].clone().unwrap());
                pairs.push((i, j));
            } else {
                checks.unknown(
                    &format!("{kind}:{}", a_id(&a[i])),
                    "Feature correspondence is ambiguous; a feature cannot be used twice",
                );
            }
        } else {
            checks.unknown(
                &format!("{kind}:{}", a_id(&a[i])),
                "Several complementary features are possible; correspondence is not established",
            );
        }
    }
    for (j, y) in b.iter().enumerate() {
        if !pairs.iter().any(|p| p.1 == j) {
            let possible = candidates
                .iter()
                .any(|row| row[j].as_ref().is_some_and(|c| c.status != "fail"));
            let comparable = a.is_empty() || candidates.iter().any(|row| row[j].is_some());
            checks.add(
                format!("{kind}:{}", b_id(y)),
                if a_complete && !possible && comparable {
                    "fail"
                } else {
                    "unknown"
                },
                format!(
                    "{kind} `{}` lacks a unique established counterpart",
                    b_id(y)
                ),
                Value::Null,
            );
        }
    }
    pairs
}
