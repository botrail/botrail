use super::*;

pub(super) fn review(
    a: Face<'_>,
    b: Face<'_>,
    ga: &MountGeometry,
    gb: &MountGeometry,
    pairs: Option<&[(usize, usize)]>,
    checks: &mut Checks,
) {
    let Some(pairs) = pairs else {
        checks.unknown(
            "mating_plane",
            "Fasteners cannot be evaluated across unseated mating planes",
        );
        return;
    };
    if !ga.complete
        || !gb.complete
        || pairs.len() != ga.holes.len()
        || pairs.len() != gb.holes.len()
    {
        checks.unknown(
            "coverage",
            "Not every required fastening position has an established counterpart",
        );
    }
    if pairs.is_empty() {
        checks.unknown(
            "coverage",
            "No planar clearance-to-threaded fastening pairs are declared",
        );
    }
    for &(i, j) in pairs {
        let x = &ga.holes[i];
        let y = &gb.holes[j];
        let (clearance, threaded, side) = if x.kind == HoleKind::Clearance {
            (x, y, a)
        } else {
            (y, x, b)
        };
        let Some(screw) = side
            .face
            .fasteners
            .iter()
            .find(|f| f.holes.contains(&clearance.id))
        else {
            checks.unknown(
                &format!("{}:selection", clearance.id),
                "The actual screw, washer and tightening selection is missing",
            );
            continue;
        };
        if !side.part.supported(&screw.evidence) {
            checks.unknown(
                &format!("{}:evidence", clearance.id),
                "The fastener selection has no supporting source",
            );
            continue;
        }
        let mut one = Checks::default();
        match &threaded.thread {
            Some(thread) => thread_check(&screw.thread, thread, "hole_thread", &mut one),
            None => one.unknown(
                "hole_thread",
                "The tapped hole's thread specification is missing",
            ),
        }
        let rules: Vec<_> = [clearance, threaded]
            .into_iter()
            .filter_map(|h| h.fastener_rules.as_ref())
            .collect();
        for (index, rule) in rules.iter().enumerate() {
            let label = format!("rule{index}");
            if let Some(thread) = &rule.thread {
                thread_check(&screw.thread, thread, &format!("{label}:thread"), &mut one);
            }
            for (key, value, wanted) in [
                ("head_standard", &screw.head_standard, &rule.head_standard),
                (
                    "property_class",
                    &screw.property_class,
                    &rule.property_class,
                ),
            ] {
                if let Some(wanted) = wanted {
                    one.add(
                        format!("{label}:{key}"),
                        if value == wanted { "pass" } else { "fail" },
                        format!("{key}: `{value}` compared with required `{wanted}`"),
                        json!({"actual": value, "required": wanted}),
                    );
                }
            }
            for (key, value, wanted) in [
                ("length_mm", screw.length_mm, rule.length_mm),
                ("torque_nm", screw.torque_nm, rule.torque_nm),
            ] {
                if let Some(wanted) = wanted {
                    one.add(
                        format!("{label}:{key}"),
                        contained(value, wanted),
                        format!("{key} selection compared across its full bounds"),
                        json!({"actual": value, "allowed": wanted}),
                    );
                }
            }
        }
        for (key, known) in [
            (
                "head_standard",
                rules.iter().any(|r| r.head_standard.is_some()),
            ),
            (
                "property_class",
                rules.iter().any(|r| r.property_class.is_some()),
            ),
            ("torque_nm", rules.iter().any(|r| r.torque_nm.is_some())),
            (
                "min_engagement_mm",
                rules.iter().any(|r| r.min_engagement_mm.is_some()),
            ),
        ] {
            if !known {
                one.unknown(key, &format!("Required {key} is not documented"));
            }
        }
        if let Some(grip) = clearance.grip_mm {
            let penetration = DimensionRange {
                min: screw.length_mm.min - grip.max - screw.washer_mm.max,
                max: screw.length_mm.max - grip.min - screw.washer_mm.min,
            };
            let lead = threaded
                .thread_start_mm
                .unwrap_or(DimensionRange { min: 0.0, max: 0.0 });
            let engagement = DimensionRange {
                min: penetration.min - lead.max,
                max: penetration.max - lead.min,
            };
            for (index, required) in rules.iter().filter_map(|r| r.min_engagement_mm).enumerate() {
                one.add(
                    format!("engagement:{index}"),
                    below(
                        DimensionRange {
                            min: required,
                            max: required,
                        },
                        engagement,
                    ),
                    "Thread engagement = length - grip - washer - unthreaded lead",
                    json!({"engagement_mm": engagement, "minimum_mm": required}),
                );
            }
            match threaded.depth_mm {
                Some(depth) => one.add(
                    "bottoming",
                    below(penetration, depth),
                    "Screw tip depth compared with usable depth including bottom allowance",
                    json!({"penetration_mm": penetration, "usable_depth_mm": depth}),
                ),
                None => one.unknown(
                    "bottoming",
                    "Usable tapped-hole depth including bottom allowance is missing",
                ),
            }
        } else {
            one.unknown("grip", "Bearing-surface-to-mating-face thickness is missing; penetration and engagement are unknown");
        }
        checks.add(format!("screw:{}", clearance.id), one.status(),
            format!("Screw at `{}`: {}", clearance.id, one.0.iter().filter(|c| c.status != "pass").map(|c| c.message.as_str()).collect::<Vec<_>>().join("; ")),
            json!({"selection": screw, "clearance_hole": clearance, "threaded_hole": threaded, "checks": one.0}));
    }
}

fn thread_check(actual: &MountThread, required: &MountThread, label: &str, checks: &mut Checks) {
    let status = if (actual.diameter_mm - required.diameter_mm).abs() > EPS
        || actual.left_hand != required.left_hand
    {
        "fail"
    } else {
        match (actual.pitch_mm, required.pitch_mm) {
            (Some(a), Some(b)) if (a - b).abs() > EPS => "fail",
            (Some(_), Some(_)) => "pass",
            _ => "unknown",
        }
    };
    checks.add(
        label,
        status,
        "Nominal thread diameter, pitch and handedness compared",
        json!({"actual": actual, "required": required}),
    );
}
