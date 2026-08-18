//! The cell report — one sheet that gathers what the other tools measure:
//! cycle times and step spans (the bake), the tightest clearance (the
//! verifier), I/O counts and findings (the map), the scenario matrix, the
//! BOM's totals, the plan-view footprint, and the hashes of the
//! deliverables written from the same source.
//!
//! It is a *reading* surface, not a judging one: the same numbers a CI
//! run asserts on with pytest, laid out for the person approving the
//! cell and for the agent iterating on it. Every field is JSON, so a
//! script can `assert report.footprint["area"] <= 20`; the Markdown is
//! the same data for humans. Nothing in here decides pass or fail — that
//! stays with the `assert`.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use serde::Serialize;

use botrail_model::RobotSource;

use crate::iomap::{self, PointStatus};
use crate::rollout::SequenceTimeline;
use crate::verify::Clearance;
use crate::Scene;

/// A robot of the cell, as the report names it.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RobotSummary {
    pub name: String,
    pub dof: usize,
    /// World position of the base.
    pub base: [f64; 3],
    pub catalog: Option<String>,
    pub manufacturer: Option<String>,
    pub model: Option<String>,
    /// Catalog `reach_mm` in metres, when declared.
    pub reach: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct StepRow {
    pub name: String,
    pub sequence: String,
    pub start: f64,
    pub end: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RobotUse {
    pub robot: String,
    pub busy: f64,
    /// Fraction of the cycle spent moving, 0..1.
    pub utilization: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ClearanceRow {
    pub distance: f64,
    pub t: f64,
    pub pair: Option<(String, String)>,
}

/// One baked cycle.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CycleSummary {
    pub name: String,
    pub sequences: Vec<String>,
    pub scenario: Option<String>,
    pub duration: f64,
    pub steps: Vec<StepRow>,
    pub robots: Vec<RobotUse>,
    pub clearance: Option<ClearanceRow>,
    /// `(sequence, step, arm)` — the path taken through branching steps.
    pub branches: Vec<(String, String, usize)>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct NodeUsage {
    pub name: String,
    pub kind: String,
    /// Points bound onto this node.
    pub bound: usize,
    /// Channels the node declares.
    pub channels: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FindingRow {
    pub severity: String,
    pub code: String,
    pub message: String,
}

/// The I/O map, summarised: point counts by kind and status, node usage,
/// and the lint findings.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct IoSummary {
    /// The program set the derivation ran over.
    pub sequences: Vec<String>,
    /// Real points (cosmetic magazine points left out).
    pub points: usize,
    pub by_kind: BTreeMap<String, usize>,
    pub bound: usize,
    pub unbound: usize,
    pub internal: usize,
    pub safety: usize,
    pub nodes: Vec<NodeUsage>,
    pub findings: Vec<FindingRow>,
}

/// One row of the scenario matrix.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ScenarioRow {
    pub name: String,
    pub ok: bool,
    pub duration: Option<f64>,
    pub error: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BomSummary {
    pub rows: usize,
    pub unidentified: usize,
    /// Quantity per category.
    pub by_category: BTreeMap<String, u32>,
    /// Σ qty × attribute for every numeric attribute any row carries.
    pub totals: BTreeMap<String, f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FootprintSummary {
    pub min: [f64; 2],
    pub max: [f64; 2],
    pub width: f64,
    pub depth: f64,
    /// Plan-view area of the bounding rectangle, m².
    pub area: f64,
    /// Tallest non-ground item, m.
    pub height: f64,
}

/// A file written from the same source as this report, with its digest —
/// the evidence that the drawing, the list and the program are one cell.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Deliverable {
    pub path: String,
    pub sha256: Option<String>,
    pub bytes: Option<u64>,
}

/// Every optional field serializes as `null` rather than disappearing:
/// the JSON keeps one shape whether or not a bake was supplied, which is
/// what a script (or an agent) reading it wants.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CellReport {
    pub title: String,
    pub robots: Vec<RobotSummary>,
    pub cycles: Vec<CycleSummary>,
    pub io: Option<IoSummary>,
    /// Why the I/O map could not be derived, when it could not.
    pub io_error: Option<String>,
    pub scenarios: Vec<ScenarioRow>,
    pub bom: BomSummary,
    pub footprint: FootprintSummary,
    pub deliverables: Vec<Deliverable>,
}

/// One baked cycle handed to the report, with the clearance already
/// measured against the scene it was baked from (the caller owns that
/// snapshot).
pub struct CycleInput<'a> {
    pub name: String,
    pub timeline: &'a SequenceTimeline,
    pub clearance: Option<Clearance>,
}

pub struct CellReportInput<'a> {
    pub title: Option<String>,
    pub cycles: Vec<CycleInput<'a>>,
    pub scenarios: Vec<ScenarioRow>,
    pub deliverables: Vec<Deliverable>,
    /// Ground threshold for the footprint (see [`crate::layout`]).
    pub ground_z: f64,
}

impl Scene {
    /// Gathers the report over this scene (see the module docs).
    pub fn cell_report(&self, input: CellReportInput<'_>) -> CellReport {
        let robots = self
            .robots()
            .iter()
            .map(|r| {
                let base = r.base_pose().translation.vector;
                let (catalog, manufacturer, model, reach) = catalog_identity(&r.model.source);
                RobotSummary {
                    name: r.name.clone(),
                    dof: r.model.dof(),
                    base: [base.x, base.y, base.z],
                    catalog,
                    manufacturer,
                    model,
                    reach,
                }
            })
            .collect();

        let cycles = input
            .cycles
            .iter()
            .map(|c| {
                let tl = c.timeline;
                CycleSummary {
                    name: c.name.clone(),
                    sequences: tl.sequences.clone(),
                    scenario: tl.scenario.clone(),
                    duration: tl.duration,
                    steps: tl
                        .step_spans
                        .iter()
                        .map(|s| StepRow {
                            name: s.name.clone(),
                            sequence: s.sequence.clone(),
                            start: s.start,
                            end: s.end,
                        })
                        .collect(),
                    robots: tl
                        .robots
                        .iter()
                        .map(|r| RobotUse {
                            robot: r.name.clone(),
                            busy: tl.busy_seconds(&r.name).unwrap_or(0.0),
                            utilization: tl.utilization(&r.name).unwrap_or(0.0),
                        })
                        .collect(),
                    clearance: c.clearance.as_ref().map(|cl| ClearanceRow {
                        distance: cl.distance,
                        t: cl.t,
                        pair: cl.pair.clone(),
                    }),
                    branches: tl
                        .branches
                        .iter()
                        .map(|b| (b.sequence.clone(), b.step.clone(), b.arm))
                        .collect(),
                }
            })
            .collect();

        let (io, io_error) = match iomap::derive(self, None) {
            Ok(d) => (Some(io_summary(&d)), None),
            Err(e) => (None, Some(e.to_string())),
        };

        let bom = self.bom();
        let mut by_category: BTreeMap<String, u32> = BTreeMap::new();
        for row in &bom.rows {
            *by_category.entry(row.category.clone()).or_default() += row.qty;
        }
        let totals: BTreeMap<String, f64> = bom
            .attribute_keys()
            .into_iter()
            .filter_map(|k| bom.total(&k).map(|t| (k, t)))
            .collect();
        let bom_summary = BomSummary {
            rows: bom.rows.len(),
            unidentified: bom.unidentified().len(),
            by_category,
            totals,
        };

        let fp = self.footprint(input.ground_z);
        let footprint = FootprintSummary {
            min: fp.min,
            max: fp.max,
            width: fp.width(),
            depth: fp.depth(),
            area: fp.area(),
            height: fp.height,
        };

        let title = input.title.clone().unwrap_or_else(|| {
            self.robots()
                .first()
                .map(|r| format!("{} cell", r.name))
                .unwrap_or_else(|| "cell".to_string())
        });

        CellReport {
            title,
            robots,
            cycles,
            io,
            io_error,
            scenarios: input.scenarios,
            bom: bom_summary,
            footprint,
            deliverables: input.deliverables,
        }
    }
}

fn catalog_identity(
    source: &RobotSource,
) -> (Option<String>, Option<String>, Option<String>, Option<f64>) {
    match source {
        RobotSource::Catalog {
            id, revision, meta, ..
        } => (
            Some(format!("{id}@{revision}")),
            meta.manufacturer.clone(),
            meta.product.clone(),
            meta.specs
                .iter()
                .find(|(k, _)| k == "reach_mm")
                .map(|(_, v)| v / 1000.0),
        ),
        RobotSource::Composite { base, .. } => catalog_identity(base),
        _ => (None, None, None, None),
    }
}

fn io_summary(d: &iomap::IoDerivation) -> IoSummary {
    let mut by_kind: BTreeMap<String, usize> = BTreeMap::new();
    let (mut points, mut bound, mut unbound, mut internal, mut safety) = (0, 0, 0, 0, 0);
    for p in &d.points {
        if matches!(p.status, PointStatus::Cosmetic) {
            continue;
        }
        points += 1;
        *by_kind.entry(p.kind.as_str().to_string()).or_default() += 1;
        if p.safety {
            safety += 1;
        }
        match p.status {
            PointStatus::Bound(_) => bound += 1,
            PointStatus::Unbound => unbound += 1,
            PointStatus::Internal => internal += 1,
            _ => {}
        }
    }
    let nodes =
        d.io.nodes
            .iter()
            .map(|n| NodeUsage {
                name: n.name.clone(),
                kind: n.kind.as_str().to_string(),
                bound: d.io.bindings.iter().filter(|b| b.node == n.name).count(),
                channels: n.channels.len(),
            })
            .collect();
    let findings = d
        .report
        .findings
        .iter()
        .map(|f| FindingRow {
            severity: f.severity.as_str().to_string(),
            code: f.code.as_str().to_string(),
            message: f.message.clone(),
        })
        .collect();
    IoSummary {
        sequences: d.sequences.clone(),
        points,
        by_kind,
        bound,
        unbound,
        internal,
        safety,
        nodes,
        findings,
    }
}

fn num(v: f64) -> String {
    if v.fract() == 0.0 && v.abs() < 1e15 {
        format!("{}", v as i64)
    } else {
        let s = format!("{v:.3}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

impl CellReport {
    /// The cycle time of the named cycle (or the first when `None`).
    pub fn cycle_time(&self, name: Option<&str>) -> Option<f64> {
        match name {
            Some(n) => self.cycles.iter().find(|c| c.name == n).map(|c| c.duration),
            None => self.cycles.first().map(|c| c.duration),
        }
    }

    /// The tightest clearance over all cycles that measured one.
    pub fn min_clearance(&self) -> Option<f64> {
        self.cycles
            .iter()
            .filter_map(|c| c.clearance.as_ref().map(|cl| cl.distance))
            .fold(None, |acc, d| Some(acc.map_or(d, |a: f64| a.min(d))))
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("report serializes")
    }

    /// The report as a Markdown document: a summary table, then one
    /// section per area.
    pub fn to_markdown(&self) -> String {
        let mut out = String::new();
        let _ = writeln!(out, "# {} — cell report\n", self.title);

        // ---- summary ---------------------------------------------------
        out.push_str("| | |\n|---|---|\n");
        let robots: Vec<String> = self
            .robots
            .iter()
            .map(|r| {
                let ident = [r.manufacturer.as_deref(), r.model.as_deref()]
                    .into_iter()
                    .flatten()
                    .collect::<Vec<_>>()
                    .join(" ");
                if ident.is_empty() {
                    format!("{} ({} DOF)", r.name, r.dof)
                } else {
                    format!("{} ({ident}, {} DOF)", r.name, r.dof)
                }
            })
            .collect();
        let _ = writeln!(out, "| Robots | {} |", robots.join(", "));
        if self.cycles.is_empty() {
            out.push_str("| Cycle time | — (no bake supplied) |\n");
        } else {
            let cycles: Vec<String> = self
                .cycles
                .iter()
                .map(|c| format!("{}: {:.2} s", c.name, c.duration))
                .collect();
            let _ = writeln!(out, "| Cycle time | {} |", cycles.join(", "));
        }
        match self
            .cycles
            .iter()
            .filter_map(|c| c.clearance.as_ref().map(|cl| (c, cl)))
            .min_by(|a, b| a.1.distance.partial_cmp(&b.1.distance).unwrap())
        {
            Some((c, cl)) => {
                let _ = writeln!(
                    out,
                    "| Min clearance | {:.3} m at {:.2} s ({}){} |",
                    cl.distance,
                    cl.t,
                    c.name,
                    cl.pair
                        .as_ref()
                        .map(|(a, b)| format!(" — {a} / {b}"))
                        .unwrap_or_default()
                );
            }
            None => out.push_str("| Min clearance | — |\n"),
        }
        let _ = writeln!(
            out,
            "| Footprint | {:.2} × {:.2} m ({:.1} m²), height {:.2} m |",
            self.footprint.width, self.footprint.depth, self.footprint.area, self.footprint.height
        );
        match &self.io {
            Some(io) => {
                let kinds: Vec<String> =
                    io.by_kind.iter().map(|(k, n)| format!("{n} {k}")).collect();
                let errors = io.findings.iter().filter(|f| f.severity == "error").count();
                let _ = writeln!(
                    out,
                    "| I/O | {} points ({}), {} unbound, {} finding(s){} |",
                    io.points,
                    if kinds.is_empty() {
                        "none".to_string()
                    } else {
                        kinds.join(", ")
                    },
                    io.unbound,
                    io.findings.len(),
                    if errors > 0 {
                        format!(", **{errors} error(s)**")
                    } else {
                        String::new()
                    }
                );
            }
            None => {
                let _ = writeln!(
                    out,
                    "| I/O | not derived: {} |",
                    self.io_error.as_deref().unwrap_or("unknown error")
                );
            }
        }
        let totals: Vec<String> = self
            .bom
            .totals
            .iter()
            .map(|(k, v)| format!("{k} {}", num(*v)))
            .collect();
        let _ = writeln!(
            out,
            "| BOM | {} lines, {} unidentified{} |",
            self.bom.rows,
            self.bom.unidentified,
            if totals.is_empty() {
                String::new()
            } else {
                format!(", {}", totals.join(", "))
            }
        );
        if !self.scenarios.is_empty() {
            let ok = self.scenarios.iter().filter(|s| s.ok).count();
            let _ = writeln!(out, "| Scenarios | {ok}/{} passed |", self.scenarios.len());
        }
        if !self.deliverables.is_empty() {
            let _ = writeln!(
                out,
                "| Deliverables | {} files hashed |",
                self.deliverables.len()
            );
        }

        // ---- cycles ----------------------------------------------------
        for c in &self.cycles {
            let _ = writeln!(
                out,
                "\n## Cycle `{}`{}\n",
                c.name,
                c.scenario
                    .as_ref()
                    .map(|s| format!(" (scenario `{s}`)"))
                    .unwrap_or_default()
            );
            let _ = writeln!(
                out,
                "Programs: {}. Duration **{:.2} s**.\n",
                c.sequences
                    .iter()
                    .map(|s| format!("`{s}`"))
                    .collect::<Vec<_>>()
                    .join(", "),
                c.duration
            );
            if !c.robots.is_empty() {
                out.push_str("| robot | busy (s) | utilization |\n|---|---|---|\n");
                for r in &c.robots {
                    let _ = writeln!(
                        out,
                        "| {} | {:.2} | {:.0} % |",
                        r.robot,
                        r.busy,
                        r.utilization * 100.0
                    );
                }
                out.push('\n');
            }
            if let Some(cl) = &c.clearance {
                let _ = writeln!(
                    out,
                    "Min clearance {:.3} m at {:.2} s{}.\n",
                    cl.distance,
                    cl.t,
                    cl.pair
                        .as_ref()
                        .map(|(a, b)| format!(" ({a} / {b})"))
                        .unwrap_or_default()
                );
            }
            if !c.branches.is_empty() {
                let path: Vec<String> = c
                    .branches
                    .iter()
                    .map(|(s, step, arm)| format!("{s}/{step} → arm {arm}"))
                    .collect();
                let _ = writeln!(out, "Branches taken: {}.\n", path.join("; "));
            }
            out.push_str("| step | start (s) | end (s) |\n|---|---|---|\n");
            for s in &c.steps {
                let name = if c.sequences.len() > 1 {
                    format!("{}/{}", s.sequence, s.name)
                } else {
                    s.name.clone()
                };
                let _ = writeln!(out, "| {name} | {:.2} | {:.2} |", s.start, s.end);
            }
        }

        // ---- I/O -------------------------------------------------------
        if let Some(io) = &self.io {
            out.push_str("\n## I/O\n\n");
            let _ = writeln!(
                out,
                "{} points over {}: {} bound, {} unbound, {} internal, {} safety.\n",
                io.points,
                io.sequences
                    .iter()
                    .map(|s| format!("`{s}`"))
                    .collect::<Vec<_>>()
                    .join(", "),
                io.bound,
                io.unbound,
                io.internal,
                io.safety
            );
            if !io.by_kind.is_empty() {
                out.push_str("| kind | points |\n|---|---|\n");
                for (k, n) in &io.by_kind {
                    let _ = writeln!(out, "| {k} | {n} |");
                }
                out.push('\n');
            }
            if !io.nodes.is_empty() {
                out.push_str("| node | kind | bound / channels |\n|---|---|---|\n");
                for n in &io.nodes {
                    let _ = writeln!(
                        out,
                        "| {} | {} | {} / {} |",
                        n.name, n.kind, n.bound, n.channels
                    );
                }
                out.push('\n');
            }
            if !io.findings.is_empty() {
                out.push_str("Findings:\n\n");
                for f in &io.findings {
                    let _ = writeln!(out, "- **{}** `{}`: {}", f.severity, f.code, f.message);
                }
            }
        } else if let Some(err) = &self.io_error {
            let _ = writeln!(out, "\n## I/O\n\nNot derived: {err}\n");
        }

        // ---- scenarios -------------------------------------------------
        if !self.scenarios.is_empty() {
            out.push_str("\n## Scenarios\n\n| scenario | result | cycle (s) |\n|---|---|---|\n");
            for s in &self.scenarios {
                let _ = writeln!(
                    out,
                    "| {} | {} | {} |",
                    s.name,
                    if s.ok {
                        "ok".to_string()
                    } else {
                        format!("**failed** — {}", s.error.as_deref().unwrap_or(""))
                    },
                    s.duration.map(|d| format!("{d:.2}")).unwrap_or_default()
                );
            }
        }

        // ---- BOM -------------------------------------------------------
        out.push_str("\n## Bill of materials\n\n");
        let _ = writeln!(
            out,
            "{} lines, {} unidentified.\n",
            self.bom.rows, self.bom.unidentified
        );
        if !self.bom.by_category.is_empty() {
            out.push_str("| category | qty |\n|---|---|\n");
            for (k, n) in &self.bom.by_category {
                let _ = writeln!(out, "| {k} | {n} |");
            }
            out.push('\n');
        }
        if !self.bom.totals.is_empty() {
            let totals: Vec<String> = self
                .bom
                .totals
                .iter()
                .map(|(k, v)| format!("{k} = {}", num(*v)))
                .collect();
            let _ = writeln!(out, "Totals: {}.\n", totals.join(", "));
        }

        // ---- footprint -------------------------------------------------
        out.push_str("\n## Footprint\n\n");
        let _ = writeln!(
            out,
            "x {:.2} … {:.2} m, y {:.2} … {:.2} m — {:.2} × {:.2} m, {:.2} m², tallest item {:.2} m.\n",
            self.footprint.min[0],
            self.footprint.max[0],
            self.footprint.min[1],
            self.footprint.max[1],
            self.footprint.width,
            self.footprint.depth,
            self.footprint.area,
            self.footprint.height
        );

        // ---- deliverables ----------------------------------------------
        if !self.deliverables.is_empty() {
            out.push_str("\n## Deliverables\n\n| file | bytes | sha256 |\n|---|---|---|\n");
            for d in &self.deliverables {
                let _ = writeln!(
                    out,
                    "| {} | {} | {} |",
                    d.path,
                    d.bytes.map(|b| b.to_string()).unwrap_or_default(),
                    d.sha256.as_deref().unwrap_or("")
                );
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::part::{Part, PartAttr};
    use botrail_model::{Geometry, RobotModel};
    use nalgebra::{Isometry3, Vector3};
    use std::sync::Arc;

    const URDF: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="l1"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>"#;

    #[test]
    fn report_gathers_bom_footprint_and_io_without_a_bake() {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(URDF).unwrap()));
        scene
            .add_obstacle(
                "table",
                Geometry::Box {
                    size: Vector3::new(1.0, 0.5, 0.7),
                },
                Isometry3::translation(1.0, 0.0, 0.35),
            )
            .unwrap();
        let mut part = Part {
            model: Some("T-1".into()),
            ..Part::default()
        };
        part.attributes
            .insert("mass_kg".into(), PartAttr::Number(30.0));
        scene.set_part("table", None, part).unwrap();
        let report = scene.cell_report(CellReportInput {
            title: None,
            cycles: Vec::new(),
            scenarios: vec![ScenarioRow {
                name: "baseline".into(),
                ok: true,
                duration: Some(1.5),
                error: None,
            }],
            deliverables: vec![Deliverable {
                path: "bom.csv".into(),
                sha256: Some("abc".into()),
                bytes: Some(120),
            }],
            ground_z: 0.02,
        });
        assert_eq!(report.title, "arm cell");
        assert_eq!(report.robots.len(), 1);
        assert_eq!(report.bom.rows, 2); // the robot line + the table
        assert_eq!(report.bom.unidentified, 1);
        assert_eq!(report.bom.totals.get("mass_kg"), Some(&30.0));
        assert!((report.footprint.width - 1.5).abs() < 1e-9);
        assert!((report.footprint.area - 0.75).abs() < 1e-9);
        assert!(report.io.is_some(), "{:?}", report.io_error);
        assert_eq!(report.cycle_time(None), None);
        assert_eq!(report.min_clearance(), None);
        let md = report.to_markdown();
        assert!(md.starts_with("# arm cell — cell report\n"), "{md}");
        assert!(md.contains("| Cycle time | — (no bake supplied) |"));
        assert!(
            md.contains("| BOM | 2 lines, 1 unidentified, mass_kg 30 |"),
            "{md}"
        );
        assert!(md.contains("| Scenarios | 1/1 passed |"));
        assert!(md.contains("| bom.csv | 120 | abc |"));
        let json: serde_json::Value = serde_json::from_str(&report.to_json()).unwrap();
        assert_eq!(json["footprint"]["area"], 0.75);
        assert_eq!(json["bom"]["by_category"]["part"], 1);
        assert_eq!(json["deliverables"][0]["sha256"], "abc");
    }
}
