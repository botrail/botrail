//! Bounded checks on declared planar faces and conservative box envelopes.
//! All geometry is in millimetres; Composite poses are converted from metres.

use botrail_model::mounting::*;
use nalgebra::{Isometry3, Point3, Vector3};
use parry3d_f64::{query, shape::Cuboid};
use serde_json::{json, Value};

use super::{MountingReport, Part};
use crate::wire::PoseMsg;

const EPS: f64 = 1e-7; // mm, numerical roundoff only

#[derive(Default)]
struct Checks(Vec<Value>);

impl Checks {
    fn add(&mut self, name: &str, status: &str, message: &str, inputs: Value) {
        self.0
            .push(json!({"check": name, "status": status, "message": message, "inputs": inputs}));
    }
    fn status(&self) -> &'static str {
        if self.0.iter().any(|v| v["status"] == "fail") {
            "fail"
        } else if self.0.is_empty() || self.0.iter().any(|v| v["status"] == "unknown") {
            "unknown"
        } else {
            "pass"
        }
    }
}

fn iso_mm(pose: &PoseMsg) -> Isometry3<f64> {
    let mut iso: Isometry3<f64> = pose.into();
    iso.translation.vector *= 1000.0;
    iso
}

fn point(position: [f64; 2], pose: &Isometry3<f64>) -> Point3<f64> {
    pose * Point3::new(position[0], position[1], 0.0)
}

fn thread_status(a: &MountThread, b: &MountThread) -> &'static str {
    if (a.diameter_mm - b.diameter_mm).abs() >= EPS || a.left_hand != b.left_hand {
        "fail"
    } else {
        a.pitch_mm.zip(b.pitch_mm).map_or("unknown", |(a, b)| {
            if (a - b).abs() < EPS {
                "pass"
            } else {
                "fail"
            }
        })
    }
}

// Worst-case interval comparison. A partial overlap is unresolved, not proof
// that every permitted manufactured/selected value satisfies the requirement.
fn within(actual: DimensionRange, allowed: DimensionRange) -> &'static str {
    if actual.min + EPS >= allowed.min && actual.max <= allowed.max + EPS {
        "pass"
    } else if actual.max < allowed.min - EPS || actual.min > allowed.max + EPS {
        "fail"
    } else {
        "unknown"
    }
}

fn hole_fit(a: &MountHole, b: &MountHole, pose: &Isometry3<f64>) -> &'static str {
    let (clearance, threaded) = match (a.kind, b.kind) {
        (HoleKind::Clearance, HoleKind::Threaded) => (a, b),
        (HoleKind::Threaded, HoleKind::Clearance) => (b, a),
        _ => return "fail", // This bolted-joint contract requires a clearance side.
    };
    let Some(diameter) = clearance.diameter_mm else {
        return "unknown";
    };
    let Some(thread) = &threaded.thread else {
        return "unknown";
    };
    if diameter.max < thread.diameter_mm - EPS {
        return "fail";
    }
    let distance =
        (point(a.position_mm, &Isometry3::identity()) - point(b.position_mm, pose)).norm();
    let Some(tolerance) = a
        .position_tolerance_mm
        .zip(b.position_tolerance_mm)
        .map(|(a, b)| a + b)
    else {
        return "unknown";
    };
    let minimum_clearance = (diameter.min - thread.diameter_mm) / 2.0;
    let maximum_clearance = (diameter.max - thread.diameter_mm) / 2.0;
    if distance + tolerance <= minimum_clearance + EPS {
        "pass"
    } else if distance - tolerance > maximum_clearance + EPS {
        "fail"
    } else {
        "unknown"
    }
}

// Maximum bipartite matching, trying proven pairs before unresolved ones.
// Feature IDs are local names; ordering and names never establish mating.
fn match_features(matrix: &[Vec<&str>], allow_unknown: bool) -> Option<Vec<usize>> {
    fn visit(
        row: usize,
        matrix: &[Vec<&str>],
        unknown: bool,
        used: &mut [bool],
        owner: &mut [Option<usize>],
    ) -> bool {
        for status in ["pass", "unknown"] {
            if status == "unknown" && !unknown {
                continue;
            }
            for col in 0..owner.len() {
                if used[col] || matrix[row][col] != status {
                    continue;
                }
                used[col] = true;
                if owner[col].is_none_or(|other| visit(other, matrix, unknown, used, owner)) {
                    owner[col] = Some(row);
                    return true;
                }
            }
        }
        false
    }
    let n = matrix.len();
    if n == 0 || matrix.iter().any(|row| row.len() != n) {
        return None;
    }
    let mut owner = vec![None; n];
    for row in 0..n {
        if !visit(row, matrix, allow_unknown, &mut vec![false; n], &mut owner) {
            return None;
        }
    }
    let mut pairs = vec![0; n];
    for (col, row) in owner.into_iter().enumerate() {
        pairs[row?] = col;
    }
    Some(pairs)
}

fn locator_fit(a: &MountLocator, b: &MountLocator, pose: &Isometry3<f64>) -> &'static str {
    let (male, female) = match (a.kind, b.kind) {
        (LocatorKind::Boss, LocatorKind::Recess) | (LocatorKind::Pin, LocatorKind::Hole) => (a, b),
        (LocatorKind::Recess, LocatorKind::Boss) | (LocatorKind::Hole, LocatorKind::Pin) => (b, a),
        _ => return "fail",
    };
    let Some((outer, inner)) = male.diameter_mm.zip(female.diameter_mm) else {
        return "unknown";
    };
    let Some((height, depth)) = male.depth_mm.zip(female.depth_mm) else {
        return "unknown";
    };
    let Some(tolerance) = a
        .position_tolerance_mm
        .zip(b.position_tolerance_mm)
        .map(|(a, b)| a + b)
    else {
        return "unknown";
    };
    let distance =
        (point(a.position_mm, &Isometry3::identity()) - point(b.position_mm, pose)).norm();
    if outer.min > inner.max + EPS
        || height.min > depth.max + EPS
        || distance - tolerance > (inner.max - outer.min) / 2.0 + EPS
    {
        "fail"
    } else if height.max <= depth.min + EPS
        && distance + tolerance <= (inner.min - outer.max) / 2.0 + EPS
    {
        "pass"
    } else {
        "unknown"
    }
}

fn dimensions(
    a: &MountGeometry,
    b: &MountGeometry,
    pose: &Isometry3<f64>,
    checks: &mut Checks,
) -> Option<Vec<usize>> {
    let normal = Vector3::new(0.0, 0.0, a.normal_z as f64)
        + pose.rotation * Vector3::new(0.0, 0.0, b.normal_z as f64);
    checks.add(
        "mating_plane",
        if normal.norm() <= 1e-8 && pose.translation.z.abs() <= EPS {
            "pass"
        } else {
            "fail"
        },
        "Mating planes must meet with opposite outward normals",
        json!({"normal_error": normal.norm(), "gap_mm": pose.translation.z}),
    );
    if !a.complete || !b.complete {
        checks.add(
            "coverage",
            "unknown",
            "Drawing coverage or tolerances are incomplete",
            json!({"flange_complete": a.complete, "mount_complete": b.complete}),
        );
    }
    let matrix: Vec<Vec<_>> = a
        .holes
        .iter()
        .map(|ha| b.holes.iter().map(|hb| hole_fit(ha, hb, pose)).collect())
        .collect();
    let proven = match_features(&matrix, false);
    let pairs = proven.clone().or_else(|| match_features(&matrix, true));
    let holes_status = if proven.is_some() {
        "pass"
    } else if pairs.is_some()
        || !a.complete
        || !b.complete
        || a.holes.is_empty() && b.holes.is_empty()
    {
        "unknown"
    } else {
        "fail"
    };
    checks.add(
        "hole_pattern",
        holes_status,
        "Required fastener holes paired by position, kind, diameter and positional tolerance",
        json!({"flange": a.holes, "mount": b.holes, "pair_statuses": matrix, "pairs": pairs}),
    );
    let locators: Vec<Vec<_>> = a
        .locators
        .iter()
        .map(|la| {
            b.locators
                .iter()
                .map(|lb| locator_fit(la, lb, pose))
                .collect()
        })
        .collect();
    let locator_status = if a.locators.is_empty() && b.locators.is_empty() {
        if a.complete && b.complete {
            "pass"
        } else {
            "unknown"
        }
    } else if match_features(&locators, false).is_some() {
        "pass"
    } else if match_features(&locators, true).is_some() || !a.complete || !b.complete {
        "unknown"
    } else {
        "fail"
    };
    checks.add(
        "locators",
        locator_status,
        "Declared cylindrical pilots/pins compared with mating recesses/holes",
        json!({"flange": a.locators, "mount": b.locators, "pair_statuses": locators}),
    );
    pairs
}

fn fasteners(
    a: &MountInterface,
    b: &MountInterface,
    base: &Part<'_>,
    tool: &Part<'_>,
    pairs: &[usize],
    checks: &mut Checks,
) {
    let ga = a.geometry.as_ref().expect("matched geometry");
    let gb = b.geometry.as_ref().expect("matched geometry");
    if !ga.complete || !gb.complete {
        checks.add(
            "coverage",
            "unknown",
            "Required hole/fastener quantity is not fully documented",
            json!({}),
        );
    }
    for (i, &j) in pairs.iter().enumerate() {
        let ha = &ga.holes[i];
        let hb = &gb.holes[j];
        let (clearance, receiver, selected_face, owner) = if ha.kind == HoleKind::Clearance {
            (ha, hb, a, base)
        } else {
            (hb, ha, b, tool)
        };
        let Some(selected) = selected_face
            .fasteners
            .iter()
            .find(|f| f.holes.contains(&clearance.id))
        else {
            checks.add(
                &clearance.id,
                "unknown",
                "No fastener selected for this required hole",
                json!({"hole": clearance}),
            );
            continue;
        };
        if !owner.supported(&selected.evidence) {
            checks.add(
                &clearance.id,
                "unknown",
                "Selected fastener has no supported drawing/specification source",
                json!({"selected": selected}),
            );
            continue;
        }
        let thread_status = receiver
            .thread
            .as_ref()
            .map_or("unknown", |t| thread_status(t, &selected.thread));
        checks.add(
            &format!("{}:thread", clearance.id),
            thread_status,
            "Selected screw diameter, pitch and handedness compared with receiver",
            json!({"selected": selected.thread, "receiver": receiver.thread}),
        );
        let (mut head, mut class, mut torque) = (false, false, false);
        for rules in [
            clearance.fastener_rules.as_ref(),
            receiver.fastener_rules.as_ref(),
        ]
        .into_iter()
        .flatten()
        {
            if let Some(thread) = &rules.thread {
                checks.add(
                    &format!("{}:specified_thread", clearance.id),
                    self::thread_status(&selected.thread, thread),
                    "Selected screw compared with the prescribed thread",
                    json!({"actual": selected.thread, "required": thread}),
                );
            }
            if let Some(length) = rules.length_mm {
                checks.add(
                    &format!("{}:length", clearance.id),
                    within(selected.length_mm, length),
                    "Selected screw length compared with the prescribed length",
                    json!({"actual_mm": selected.length_mm, "required_mm": length}),
                );
            }
            for (key, actual, expected) in [
                (
                    "head",
                    &selected.head_standard,
                    rules.head_standard.as_ref(),
                ),
                (
                    "property_class",
                    &selected.property_class,
                    rules.property_class.as_ref(),
                ),
            ] {
                let Some(expected) = expected else {
                    continue;
                };
                if key == "head" {
                    head = true;
                } else {
                    class = true;
                }
                checks.add(
                    &format!("{}:{key}", clearance.id),
                    if expected == actual { "pass" } else { "fail" },
                    "Selected fastener specification compared with drawing requirement",
                    json!({"actual": actual, "required": expected}),
                );
            }
            if let Some(allowed) = rules.torque_nm {
                torque = true;
                checks.add(
                    &format!("{}:torque", clearance.id),
                    within(selected.torque_nm, allowed),
                    "Selected tightening torque compared with documented limits",
                    json!({"actual_nm": selected.torque_nm, "allowed_nm": rules.torque_nm}),
                );
            }
        }
        if !head || !class || !torque {
            checks.add(
                &clearance.id,
                "unknown",
                "Required head, property class and tightening torque are undocumented",
                json!({"head_documented": head, "property_class_documented": class, "torque_documented": torque}),
            );
        }
        let engagement = clearance.grip_mm.map(|grip| DimensionRange {
            min: selected.length_mm.min - grip.max - selected.washer_mm.max,
            max: selected.length_mm.max - grip.min - selected.washer_mm.min,
        });
        let minimum = receiver
            .fastener_rules
            .as_ref()
            .and_then(|r| r.min_engagement_mm);
        let status = match (engagement, minimum, receiver.depth_mm) {
            (Some(actual), Some(min), Some(depth)) => {
                if min > depth.max + EPS || actual.max < min - EPS || actual.min > depth.max + EPS {
                    "fail"
                } else if actual.min + EPS >= min && actual.max <= depth.min + EPS {
                    "pass"
                } else {
                    "unknown"
                }
            }
            (Some(actual), _, Some(depth)) if actual.min > depth.max + EPS || actual.max <= 0.0 => {
                "fail"
            }
            (Some(actual), Some(min), _) if actual.max < min - EPS => "fail",
            _ => "unknown",
        };
        checks.add(&format!("{}:engagement", clearance.id), status, "Screw length minus bearing stack and washer thickness compared with usable thread engagement",
            json!({"length_mm": selected.length_mm, "grip_mm": clearance.grip_mm, "washer_mm": selected.washer_mm,
                "engagement_mm": engagement, "min_engagement_mm": minimum, "usable_depth_mm": receiver.depth_mm, "sources": owner.evidence(&selected.evidence)}));
    }
}

fn overlap(a: &MountBox, pa: &Isometry3<f64>, b: &MountBox, pb: &Isometry3<f64>) -> bool {
    let ta = pa * Isometry3::translation(a.center_mm[0], a.center_mm[1], a.center_mm[2]);
    let tb = pb * Isometry3::translation(b.center_mm[0], b.center_mm[1], b.center_mm[2]);
    let sa = Cuboid::new(parry3d_f64::math::Vector::from_array(a.size_mm) / 2.0);
    let sb = Cuboid::new(parry3d_f64::math::Vector::from_array(b.size_mm) / 2.0);
    // SAT handles complete containment too; a closest surface contact is not
    // a reliable penetration measure when an access box is inside a body.
    let relative = botrail_collide::to_parry_pose(&(ta.inverse() * tb));
    let face_a =
        query::sat::cuboid_cuboid_find_local_separating_normal_oneway(&sa, &sb, &relative).0;
    let face_b = query::sat::cuboid_cuboid_find_local_separating_normal_oneway(
        &sb,
        &sa,
        &relative.inverse(),
    )
    .0;
    let edges = query::sat::cuboid_cuboid_find_local_separating_edge_twoway(&sa, &sb, &relative).0;
    face_a.max(face_b).max(edges) < -EPS
}

pub(super) fn review(
    report: &mut MountingReport,
    target: &str,
    base: Option<&Part<'_>>,
    tool: Option<&Part<'_>>,
    flange: Option<&MountInterface>,
    mount: Option<&MountInterface>,
    offset: &PoseMsg,
) {
    let pose = iso_mm(offset);
    let mut dim = Checks::default();
    let mut bolts = Checks::default();
    let geometry = flange
        .and_then(|f| f.geometry.as_ref())
        .zip(mount.and_then(|f| f.geometry.as_ref()));
    let any_geometry =
        flange.is_some_and(|f| f.geometry.is_some()) || mount.is_some_and(|f| f.geometry.is_some());
    if let Some(((ga, gb), (base, tool))) = geometry.zip(base.zip(tool)) {
        if ga.frame_verified
            && gb.frame_verified
            && base.supported(&ga.evidence)
            && tool.supported(&gb.evidence)
        {
            if let Some(pairs) = dimensions(ga, gb, &pose, &mut dim) {
                // A shifted/rotated face cannot establish the bearing stack.
                if dim.0[0]["status"] == "pass" {
                    fasteners(
                        flange.unwrap(),
                        mount.unwrap(),
                        base,
                        tool,
                        &pairs,
                        &mut bolts,
                    );
                }
            }
        } else {
            dim.add("frame_evidence", "unknown", "Drawing-to-model frame correspondence or source evidence is missing", json!({"flange_frame_verified": ga.frame_verified, "mount_frame_verified": gb.frame_verified}));
        }
    } else if any_geometry {
        dim.add(
            "geometry",
            "unknown",
            "Mating geometry is missing on one side",
            json!({}),
        );
    }
    let sources = json!({"flange": geometry.zip(base).map(|((g, _), p)| p.evidence(&g.evidence)), "mount": geometry.zip(tool).map(|((_, g), p)| p.evidence(&g.evidence))});
    report.add(
        target,
        "dimensions",
        if any_geometry {
            dim.status()
        } else {
            "not_run"
        },
        "Declared mating dimensions and tolerances",
        "Resolve listed hole, locator, plane or drawing-coverage findings",
        json!({"checks": dim.0, "sources": sources, "geometry": geometry}),
    );
    let any_fasteners = any_geometry
        || flange.is_some_and(|f| !f.fasteners.is_empty())
        || mount.is_some_and(|f| !f.fasteners.is_empty());
    report.add(target, "fasteners", if any_fasteners { bolts.status() } else { "not_run" }, "Selected fasteners and documented engagement conditions",
        "Supply or correct the screw selection, bearing stack and manufacturer tightening/engagement requirements", json!({"checks": bolts.0, "sources": sources}));

    let mut space = Checks::default();
    let ca = flange.and_then(|f| f.clearance.as_ref());
    let cb = mount.and_then(|f| f.clearance.as_ref());
    if let Some(((a, b), (base, tool))) = ca.zip(cb).zip(base.zip(tool)) {
        if a.frame_verified
            && b.frame_verified
            && base.supported(&a.evidence)
            && tool.supported(&b.evidence)
        {
            if !a.complete || !b.complete {
                space.add(
                    "coverage",
                    "unknown",
                    "Conservative envelope/access coverage is incomplete",
                    json!({}),
                );
            }
            let identity = Isometry3::identity();
            for (name, one, two, p, q) in [
                ("bodies", &a.solids, &b.solids, &identity, &pose),
                ("flange_access", &a.access, &b.solids, &identity, &pose),
                ("tool_access", &b.access, &a.solids, &pose, &identity),
            ] {
                let clashes: Vec<_> = one
                    .iter()
                    .flat_map(|x| {
                        two.iter()
                            .filter(move |y| overlap(x, p, y, q))
                            .map(move |y| json!([x.id, y.id]))
                    })
                    .collect();
                space.add(
                    name,
                    if clashes.is_empty() { "pass" } else { "fail" },
                    "Conservative box envelopes compared at the attachment pose",
                    json!({"overlaps": clashes}),
                );
            }
        } else {
            space.add(
                "frame_evidence",
                "unknown",
                "Assembly envelopes/access spaces lack supported frame correspondence or evidence",
                json!({}),
            );
        }
    } else if ca.is_some() || cb.is_some() {
        space.add(
            "coverage",
            "unknown",
            "Assembly envelope/access declaration is missing on one side",
            json!({}),
        );
    }
    report.add(target, "assembly_clearance", if ca.is_some() || cb.is_some() { space.status() } else { "not_run" }, "Declared mating-part envelopes and required access volumes",
        "Provide conservative geometry and all required access spaces for this joint", json!({"checks": space.0, "scope": "Adjacent mating parts at the declared pose; external equipment and approach motion are outside this check",
            "flange": ca, "mount": cb, "flange_sources": ca.zip(base).map(|(c, p)| p.evidence(&c.evidence)), "mount_sources": cb.zip(tool).map(|(c, p)| p.evidence(&c.evidence))}));
}
