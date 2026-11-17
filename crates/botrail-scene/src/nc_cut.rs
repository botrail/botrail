//! What a machine tool's part programs cut: the stock carved along a baked
//! timeline by the tools the programs put in the spindle, and what went
//! wrong while they did.
//!
//! A robot's cutter follows its toolpath by construction, so carving its
//! stock ([`crate::carve`]) is a picture. A machine tool is four
//! independent inputs meeting at the cutter — the program, the work
//! offset, the tool table and where the part really stands — so here the
//! cut is a **verification**. The tip is where the spindle nose is (the
//! axes, as the control positioned them from the offset and the length
//! *entered* in the table) less the length the tool really *has*; the
//! stock is wherever its own track has it, clamped to a table that moves.
//! What comes out is the machined part, the material each source line
//! removed, and the things a dry run is run for: a cutter that met
//! material at rapid or with the spindle stopped, a shank or holder in
//! the stock, a tool in a fixture — each with its line — and, against a
//! design model, the material left over and the material missing.
//!
//! Like the robot's carve this is a pass over a finished bake: nothing in
//! the cell is planned against the cut shape (an arm grips the stock by
//! the faces it was taught on).
//!
//! Units: botrail works in meters, cutrail in millimeters; the conversion
//! happens here and in [`crate::carve`], nowhere else.

use std::collections::BTreeMap;

use botrail_mesh::MeshData;
use botrail_model::Geometry;
use cutrail::{
    CutOutcome, Engine, EngineOptions, Holder, ManifoldKernel, MotionSegment, PartId,
    Pose as ToolPose, SegmentBuilder, Shank, StockShape, Tool, TriMesh,
};
use nalgebra::{Isometry3, Point3, Translation3, Vector3};

use crate::carve::{colored_mesh, stock_shape, CarveError, StockCarve};
use crate::nc::{NcCutter, NcEvent, NcRun, NcToolBody, NcToolChange};
use crate::rollout::SequenceTimeline;
use crate::Scene;

/// Meters to millimeters.
const MM: f64 = 1e3;
/// Cubic meters to cubic millimeters.
const MM3: f64 = 1e9;
/// Material below this (m³ — a tenth of a cubic millimetre) is the
/// polygonization of a sweep grazing a surface it already cut, not a cut.
const GRAZE: f64 = 1e-10;

#[derive(Debug, Clone)]
pub struct MachiningOptions {
    /// Chord tolerance (m) the cutters' sweeps and round stock are
    /// polygonized with. The cut geometry is exact to this.
    pub chord_tolerance: f64,
    /// No sample of the tip's path strays farther than this (m) from the
    /// segment it was folded into.
    pub max_deviation: f64,
    /// Edge (m) of the cubic tiles a large stock is kept in, so a cut only
    /// touches the tiles under the cutter. `None` keeps the stock whole —
    /// right for a part that fits a vise.
    pub tile_size: Option<f64>,
    /// Sampling period (s) of the axes' motion.
    pub dt: f64,
    /// A snapshot of the stock is taken no more often than this (s) —
    /// the display lags the cutter by at most this.
    pub stage_period: f64,
    /// And never more snapshots than this over the whole bake.
    pub max_stages: usize,
    /// Face color of the surviving original skin (linear RGB).
    pub stock_color: [f32; 3],
    /// Face color of surfaces the cutters created (linear RGB).
    pub cut_color: [f32; 3],
}

impl Default for MachiningOptions {
    fn default() -> Self {
        MachiningOptions {
            chord_tolerance: 1e-5,
            max_deviation: 1e-5,
            tile_size: None,
            dt: 0.01,
            stage_period: 2.0,
            max_stages: 40,
            stock_color: [0.58, 0.60, 0.63],
            cut_color: [0.92, 0.94, 0.98],
        }
    }
}

/// What a tool met that it should not have.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MachiningIssueKind {
    /// The cutter met the stock at rapid.
    RapidIntoStock,
    /// The cutter met the stock with the spindle stopped.
    SpindleStopped,
    /// The shank or the holder met the stock.
    HolderIntoStock,
    /// The tool — any of it — met a fixture.
    IntoFixture,
}

impl MachiningIssueKind {
    pub fn as_str(self) -> &'static str {
        match self {
            MachiningIssueKind::RapidIntoStock => "rapid_into_stock",
            MachiningIssueKind::SpindleStopped => "spindle_stopped",
            MachiningIssueKind::HolderIntoStock => "holder_into_stock",
            MachiningIssueKind::IntoFixture => "into_fixture",
        }
    }
}

/// One thing that went wrong, on the source line it happened on.
/// Consecutive contacts of one kind on one line are one issue.
#[derive(Debug, Clone, PartialEq)]
pub struct MachiningIssue {
    pub kind: MachiningIssueKind,
    pub program: String,
    /// Source line (1-based).
    pub line: usize,
    pub tool: u32,
    /// Timeline time of the first contact (s).
    pub time: f64,
    /// Material in the way (m³).
    pub volume: f64,
    /// What was met: the stock, or the fixture's obstacle.
    pub what: String,
}

/// The material one source line removed.
#[derive(Debug, Clone, PartialEq)]
pub struct LineRemoval {
    pub program: String,
    pub line: usize,
    pub tool: u32,
    /// m³.
    pub volume: f64,
}

/// The whole stock as of `time` — one frame of the progressive display.
#[derive(Debug, Clone)]
pub struct MachiningStage {
    /// Timeline time this state is current from.
    pub time: f64,
    /// Stock-local boundary mesh; empty when everything was cut away.
    pub mesh: MeshData,
    /// Cumulative removed volume as of this stage (m³).
    pub removed_volume: f64,
}

/// The machined stock against its design model. Volumes in m³, boxes in
/// the stock's frame (m).
#[derive(Debug, Clone)]
pub struct DesignCheck {
    /// Deviations within this (m) are ignored.
    pub tolerance: f64,
    /// Material left outside the design — an undercut.
    pub excess_volume: f64,
    /// Material missing inside the design — a gouge.
    pub gouge_volume: f64,
    pub excess_bounds: Option<([f64; 3], [f64; 3])>,
    pub gouge_bounds: Option<([f64; 3], [f64; 3])>,
    /// The two regions as meshes, stock-local (empty where there is none).
    pub excess: MeshData,
    pub gouge: MeshData,
}

/// A stretch of the bake one tool stood in the spindle for.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolSpan {
    pub tool: u32,
    /// Timeline times; the tool a bake ends with stands to its end.
    pub from: f64,
    pub to: f64,
}

/// What the programs of one machine did to one stock over a bake.
#[derive(Debug, Clone)]
pub struct Machining {
    /// The machined part and the volume bookkeeping.
    pub carve: StockCarve,
    /// The stock before anything cut it, as the same kind of mesh — what
    /// a progressive display shows until the first stage.
    pub raw: MeshData,
    /// The stock as it was cut, in time order (empty when nothing cut).
    pub stages: Vec<MachiningStage>,
    /// What stood in the spindle when, in order.
    pub tools: Vec<ToolSpan>,
    /// Every tool change the axes stood for, in timeline times — where a
    /// machine with a tool changer hands a tool over to its pocket and
    /// takes the next one.
    pub changes: Vec<NcToolChange>,
    pub issues: Vec<MachiningIssue>,
    /// Removal by source line, in the order the lines first cut.
    pub lines: Vec<LineRemoval>,
    /// Timeline times of the first and the last cut.
    pub cut_span: Option<(f64, f64)>,
    /// Tools the programs called that have no body in the table: their
    /// moves cut nothing here.
    pub unbodied: Vec<u32>,
    pub design: Option<DesignCheck>,
}

/// A tool body as cutrail's tool (mm).
fn cutter(body: &NcToolBody) -> Tool {
    let (d, flute) = (body.diameter * MM, body.flute_length * MM);
    let mut tool = match body.cutter {
        // A drill cuts as a flat end here: wall and depth, not the cone.
        NcCutter::Flat | NcCutter::Drill => Tool::flat_end_mill(d, flute),
        NcCutter::Ball => Tool::ball_end_mill(d, flute),
        NcCutter::BullNose => {
            Tool::bull_nose_end_mill(d, body.corner_radius.unwrap_or(0.0) * MM, flute)
        }
    };
    let holder = body.holder.map_or(0.0, |h| h[1]);
    let shank = body.length - body.flute_length - holder;
    if shank > 1e-9 {
        tool = tool.with_shank(Shank {
            diameter: body.shank_diameter.unwrap_or(body.diameter) * MM,
            length: shank * MM,
        });
    }
    if let Some([diameter, length]) = body.holder {
        tool = tool.with_holder(Holder {
            diameter: diameter * MM,
            length: length * MM,
        });
    }
    tool
}

/// Vertices and triangles of a closed mesh (m).
type Solid = (Vec<[f64; 3]>, Vec<[u32; 3]>);

/// An obstacle's solid as a closed mesh in its own frame (m).
fn solid_mesh(name: &str, geometry: &Geometry, tolerance: f64) -> Result<Solid, CarveError> {
    match geometry {
        Geometry::Box { size } => {
            let (x, y, z) = (size.x / 2.0, size.y / 2.0, size.z / 2.0);
            let vertices = vec![
                [-x, -y, -z],
                [x, -y, -z],
                [x, y, -z],
                [-x, y, -z],
                [-x, -y, z],
                [x, -y, z],
                [x, y, z],
                [-x, y, z],
            ];
            let triangles = vec![
                [0, 2, 1],
                [0, 3, 2],
                [4, 5, 6],
                [4, 6, 7],
                [0, 1, 5],
                [0, 5, 4],
                [1, 2, 6],
                [1, 6, 5],
                [2, 3, 7],
                [2, 7, 6],
                [3, 0, 4],
                [3, 4, 7],
            ];
            Ok((vertices, triangles))
        }
        Geometry::Cylinder { radius, length } => {
            // Sides enough for the chord to stay within the tolerance.
            let step = 2.0 * (1.0 - (tolerance / radius).min(0.5)).acos();
            let n = ((std::f64::consts::TAU / step).ceil() as u32).clamp(12, 256);
            let mut vertices = Vec::with_capacity(2 * n as usize + 2);
            for z in [-length / 2.0, length / 2.0] {
                for k in 0..n {
                    let a = std::f64::consts::TAU * k as f64 / n as f64;
                    vertices.push([radius * a.cos(), radius * a.sin(), z]);
                }
            }
            vertices.push([0.0, 0.0, -length / 2.0]);
            vertices.push([0.0, 0.0, length / 2.0]);
            let (low, high) = (2 * n, 2 * n + 1);
            let mut triangles = Vec::with_capacity(4 * n as usize);
            for k in 0..n {
                let j = (k + 1) % n;
                triangles.push([low, j, k]);
                triangles.push([high, n + k, n + j]);
                triangles.push([k, j, n + j]);
                triangles.push([k, n + j, n + k]);
            }
            Ok((vertices, triangles))
        }
        Geometry::Sphere { .. } => Err(CarveError::Stock(
            name.to_string(),
            "a sphere is not a fixture shape".to_string(),
        )),
        Geometry::Mesh { path, scale } => {
            let data = botrail_mesh::load_path(path)
                .map_err(|e| CarveError::Stock(name.to_string(), e.to_string()))?;
            let vertices = data
                .vertices
                .iter()
                .map(|v| [v[0] * scale.x, v[1] * scale.y, v[2] * scale.z])
                .collect();
            Ok((vertices, data.indices))
        }
    }
}

/// A mesh in meters as cutrail's, in millimeters, moved by `to`.
fn placed_mm(vertices: &[[f64; 3]], triangles: &[[u32; 3]], to: &Isometry3<f64>) -> TriMesh {
    TriMesh {
        positions: vertices
            .iter()
            .map(|v| {
                let p = to * Point3::new(v[0], v[1], v[2]);
                [p.x * MM, p.y * MM, p.z * MM]
            })
            .collect(),
        triangles: triangles.to_vec(),
    }
}

/// The fixtures as one engine, standing where they stand from the stock:
/// whatever a tool takes out of them is a crash.
struct Fixtures {
    engine: Engine,
    /// The fixture each part of the engine is, by part id.
    names: BTreeMap<u32, String>,
}

/// What state a stretch of motion is cut in. A change of any of it ends
/// the segment being built.
#[derive(Debug, Clone, PartialEq)]
struct Mode {
    line: usize,
    rapid: bool,
    tool: u32,
    spinning: bool,
}

/// One program run on the timeline.
struct Window {
    t0: f64,
    program: String,
    run: NcRun,
}

/// The pass's running state — what a cut updates.
struct Pass<'a> {
    stock: &'a str,
    engine: Engine,
    initial_volume: f64,
    options: &'a MachiningOptions,
    period: f64,
    issues: Vec<MachiningIssue>,
    lines: Vec<LineRemoval>,
    stages: Vec<MachiningStage>,
    cut_span: Option<(f64, f64)>,
    /// Cut since the last snapshot.
    dirty: bool,
}

impl Pass<'_> {
    fn issue(
        &mut self,
        kind: MachiningIssueKind,
        program: &str,
        mode: &Mode,
        time: f64,
        volume: f64,
        what: &str,
    ) {
        if let Some(last) = self.issues.last_mut() {
            if last.kind == kind
                && last.program == program
                && last.line == mode.line
                && last.tool == mode.tool
                && last.what == what
            {
                last.volume += volume;
                return;
            }
        }
        self.issues.push(MachiningIssue {
            kind,
            program: program.to_string(),
            line: mode.line,
            tool: mode.tool,
            time,
            volume,
            what: what.to_string(),
        });
    }

    fn snapshot(&mut self, time: f64) -> Result<(), CarveError> {
        let mesh = if self.engine.parts().is_empty() {
            MeshData::new(Vec::new(), Vec::new())
        } else {
            let (mesh, kinds) = self
                .engine
                .merged_mesh_with_kinds()
                .map_err(|e| CarveError::Cut(self.stock.to_string(), e.to_string()))?;
            colored_mesh(
                &mesh,
                &kinds,
                self.options.stock_color,
                self.options.cut_color,
            )
        };
        self.stages.push(MachiningStage {
            time,
            mesh,
            removed_volume: self.initial_volume - self.engine.total_volume() / MM3,
        });
        self.dirty = false;
        Ok(())
    }

    /// Subtracts one segment of `tool`'s motion, ending at timeline time
    /// `time`, and books what it did.
    fn cut(
        &mut self,
        tool: &Tool,
        segment: &MotionSegment,
        program: &str,
        mode: &Mode,
        time: f64,
        fixtures: Option<&mut Fixtures>,
    ) -> Result<(), CarveError> {
        let failed = |e: cutrail::Error| CarveError::Cut(self.stock.to_string(), e.to_string());
        let report = self.engine.cut(tool, segment).map_err(failed)?;
        let removed = report.removed_volume / MM3;
        let holder = report.collision.as_ref().map(|c| c.volume / MM3);
        if report.outcome != CutOutcome::NoChange && removed > 0.0 {
            match self
                .lines
                .iter_mut()
                .find(|l| l.program == program && l.line == mode.line && l.tool == mode.tool)
            {
                Some(entry) => entry.volume += removed,
                None => self.lines.push(LineRemoval {
                    program: program.to_string(),
                    line: mode.line,
                    tool: mode.tool,
                    volume: removed,
                }),
            }
            self.cut_span = Some((self.cut_span.map_or(time, |(first, _)| first), time));
            self.dirty = true;
            if removed > GRAZE {
                let stock = self.stock;
                if mode.rapid {
                    self.issue(
                        MachiningIssueKind::RapidIntoStock,
                        program,
                        mode,
                        time,
                        removed,
                        stock,
                    );
                } else if !mode.spinning {
                    self.issue(
                        MachiningIssueKind::SpindleStopped,
                        program,
                        mode,
                        time,
                        removed,
                        stock,
                    );
                }
            }
        }
        if let Some(volume) = holder.filter(|v| *v > GRAZE) {
            let stock = self.stock;
            self.issue(
                MachiningIssueKind::HolderIntoStock,
                program,
                mode,
                time,
                volume,
                stock,
            );
        }
        if let Some(fixtures) = fixtures {
            let before = fixtures.engine.revision();
            let report = fixtures
                .engine
                .cut(tool, segment)
                .map_err(|e| CarveError::Cut(self.stock.to_string(), e.to_string()))?;
            let met = report.removed_volume / MM3
                + report.collision.as_ref().map_or(0.0, |c| c.volume / MM3);
            if met > GRAZE {
                // The fixture the cutter took material from, or the one
                // whose box the holder's contact lies in.
                let changed: Vec<PartId> = fixtures.engine.changed_since(before);
                let what = changed
                    .first()
                    .and_then(|id| fixtures.names.get(&id.0))
                    .or_else(|| {
                        let hit = report.collision.as_ref()?.bounds;
                        fixtures
                            .engine
                            .parts()
                            .iter()
                            .find(|part| {
                                (0..3).all(|a| {
                                    part.bounds.min[a] <= hit.max[a]
                                        && hit.min[a] <= part.bounds.max[a]
                                })
                            })
                            .and_then(|part| fixtures.names.get(&part.id.0))
                    })
                    .cloned()
                    .unwrap_or_else(|| "fixture".to_string());
                self.issue(
                    MachiningIssueKind::IntoFixture,
                    program,
                    mode,
                    time,
                    met,
                    &what,
                );
            }
        }
        if self.dirty
            && self
                .stages
                .last()
                .is_none_or(|last| time - last.time >= self.period - 1e-9)
        {
            self.snapshot(time)?;
        }
        Ok(())
    }
}

/// Tool `number` goes into the spindle at `time`: whatever stood there
/// until now stops there.
fn change_tool(spans: &mut Vec<ToolSpan>, number: u32, time: f64) {
    if let Some(last) = spans.last_mut() {
        last.to = time;
        if last.tool == number {
            last.to = f64::INFINITY;
            return;
        }
    }
    spans.push(ToolSpan {
        tool: number,
        from: time,
        to: f64::INFINITY,
    });
}

/// Carves `stock` with what the part programs of the machine `machine`
/// (a robot declared an NC machine) did over `timeline`: every program
/// run on its track is read again from where the axes stood, and the
/// tool then in the spindle — its body out of the tool table — is swept
/// from the spindle nose (`spindle`: the link whose frame is on the nose,
/// +Z up the tool) through the stock wherever the stock's own track has
/// it. `fixtures` are obstacles a tool must not touch — the vise, its
/// jaws — taken to stand still from the stock while a program runs.
/// `design` is the finished part as designed, a closed mesh in the
/// stock's frame, and the tolerance to compare within.
///
/// `scene` must be the pre-rollout snapshot the timeline was baked
/// against (same contract as `timeline_min_clearance`).
#[allow(clippy::too_many_arguments)]
pub fn machine_stock(
    scene: &Scene,
    timeline: &SequenceTimeline,
    stock: &str,
    machine: usize,
    spindle: usize,
    fixtures: &[String],
    design: Option<(&MeshData, f64)>,
    options: &MachiningOptions,
) -> Result<Machining, CarveError> {
    let positive = |v: f64| v.is_finite() && v > 0.0;
    if !(positive(options.chord_tolerance)
        && positive(options.max_deviation)
        && positive(options.dt)
        && positive(options.stage_period)
        && options.max_stages > 0
        && options.tile_size.is_none_or(positive))
    {
        return Err(CarveError::BadOptions);
    }
    let fail = |message: String| CarveError::Cut(stock.to_string(), message);
    let (obstacle, _) = scene
        .obstacle_with_collider(stock)
        .ok_or_else(|| CarveError::UnknownStock(stock.to_string()))?;
    let robot = scene
        .robots()
        .get(machine)
        .ok_or(CarveError::UnknownRobot(machine))?;
    let control = scene
        .nc_machine(&robot.name)
        .ok_or_else(|| fail(format!("`{}` is not an NC machine", robot.name)))?;
    let track = timeline
        .robots
        .get(machine)
        .ok_or(CarveError::UnknownRobot(machine))?;

    // Every program the machine ran, read again from where its axes stood:
    // the same reading the bake made, so the same lines at the same times.
    let mut windows = Vec::new();
    for run in &track.moves {
        // A run is a program, or one block of one.
        let Some((program, block)) =
            crate::nc::parse_run_name(&run.name, |name| scene.nc_program(name).is_some())
        else {
            continue;
        };
        // (A run the bake ended in the middle of is cut as far as it got.)
        if run.start >= timeline.duration - 1e-9 {
            continue;
        }
        let from = track.trajectory.sample(run.start);
        let planned = scene
            .plan_nc_program(&robot.name, program, Some(&from), block)
            .map_err(|e| fail(format!("NC program `{}`: {e}", run.name)))?;
        windows.push(Window {
            t0: run.start,
            program: program.to_string(),
            run: planned.run,
        });
    }
    windows.sort_by(|a, b| a.t0.total_cmp(&b.t0));

    let mut engine_options = EngineOptions {
        sweep_tolerance: options.chord_tolerance * MM,
        stock_tolerance: options.chord_tolerance * MM,
        ..EngineOptions::default()
    }
    .with_collision_check();
    if let Some(tile) = options.tile_size {
        engine_options = engine_options.with_tile_size(tile * MM);
    }
    let engine = Engine::new(
        ManifoldKernel,
        &[stock_shape(stock, &obstacle.geometry)?],
        engine_options.clone(),
    )
    .map_err(|e| CarveError::Stock(stock.to_string(), e.to_string()))?;
    let initial_volume = engine.total_volume() / MM3;
    let cutting: f64 = windows
        .iter()
        .map(|w| w.run.duration().min(timeline.duration - w.t0))
        .sum();
    let mut pass = Pass {
        stock,
        engine,
        initial_volume,
        options,
        period: options
            .stage_period
            .max(cutting / options.max_stages as f64),
        issues: Vec::new(),
        lines: Vec::new(),
        stages: Vec::new(),
        cut_span: None,
        dirty: false,
    };

    // Where everything stands at `t`: every robot's links, and from them
    // whatever rides one.
    let links_at = |t: f64| -> Result<Vec<Vec<Isometry3<f64>>>, CarveError> {
        timeline
            .robots
            .iter()
            .enumerate()
            .map(|(r, track)| scene.fk_for(r, &track.trajectory.sample(t)))
            .collect::<Result<_, _>>()
            .map_err(|e| fail(e.to_string()))
    };
    let stands = |name: &str, rest: Isometry3<f64>, links: &[Vec<Isometry3<f64>>], t: f64| {
        timeline
            .objects
            .iter()
            .find(|track| track.name == name)
            .and_then(|track| SequenceTimeline::span_pose(&track.spans, links, t))
            .unwrap_or(rest)
    };

    let raw = {
        let (mesh, kinds) = pass
            .engine
            .merged_mesh_with_kinds()
            .map_err(|e| fail(e.to_string()))?;
        colored_mesh(&mesh, &kinds, options.stock_color, options.cut_color)
    };
    let mut tools: BTreeMap<u32, (Tool, f64)> = BTreeMap::new();
    let mut tool_spans: Vec<ToolSpan> = Vec::new();
    let mut changes: Vec<NcToolChange> = Vec::new();
    let mut unbodied: Vec<u32> = Vec::new();
    // What is in the spindle and whether it turns carry over from one
    // program to the next, as they do on the machine.
    let (mut in_spindle, mut spinning): (Option<u32>, bool) = (None, false);
    for window in &windows {
        let run = &window.run;
        let duration = run.duration().min(timeline.duration - window.t0);
        changes.extend(
            run.changes
                .iter()
                .filter(|change| change.start < duration)
                .map(|change| NcToolChange {
                    start: window.t0 + change.start,
                    end: window.t0 + change.end,
                    tool: change.tool,
                }),
        );

        // The fixtures, where they stand from the stock as the program
        // starts — clamped together, they stay so while it runs.
        let links = links_at(window.t0)?;
        let stock_from = stands(stock, obstacle.pose, &links, window.t0).inverse();
        let mut fixture_set = if fixtures.is_empty() {
            None
        } else {
            let mut shapes = Vec::with_capacity(fixtures.len());
            for name in fixtures {
                let (fixture, _) = scene
                    .obstacle_with_collider(name)
                    .ok_or_else(|| CarveError::UnknownStock(name.clone()))?;
                let (vertices, triangles) =
                    solid_mesh(name, &fixture.geometry, options.chord_tolerance)?;
                let at = stock_from * stands(name, fixture.pose, &links, window.t0);
                shapes.push(StockShape::Mesh(placed_mm(&vertices, &triangles, &at)));
            }
            let engine = Engine::new(
                ManifoldKernel,
                &shapes,
                EngineOptions {
                    tile_size: None,
                    ..engine_options.clone()
                },
            )
            .map_err(|e| fail(format!("fixtures: {e}")))?;
            let names = engine
                .parts()
                .iter()
                .zip(fixtures)
                .map(|(part, name)| (part.id.0, name.clone()))
                .collect();
            Some(Fixtures { engine, names })
        };

        // The run's clock: the sampling grid, and every instant something
        // changes — a line, rapid to feed, a tool, the spindle — so that
        // each stretch between two instants is cut in one state.
        let steps = (duration / options.dt).ceil() as usize;
        let mut clock: Vec<f64> = (0..=steps)
            .map(|k| (k as f64 * options.dt).min(duration))
            .collect();
        clock.extend(run.spans.iter().flat_map(|s| [s.start, s.end]));
        clock.extend(run.rapids.iter().flatten().copied());
        clock.extend(run.events.iter().map(|e| e.t));
        clock.retain(|t| *t >= 0.0 && *t <= duration + 1e-12);
        clock.sort_by(f64::total_cmp);
        clock.dedup_by(|b, a| (*b - *a).abs() < 1e-9);

        // The tip in the stock's frame at run time `tau`, for a tool
        // `length` long.
        let tip = |tau: f64, length: f64| -> Result<ToolPose, CarveError> {
            let t = window.t0 + tau;
            let links = links_at(t)?;
            let nose = stands(stock, obstacle.pose, &links, t).inverse() * links[machine][spindle];
            let at = nose * Translation3::new(0.0, 0.0, -length);
            let p = at.translation.vector;
            let axis = at.rotation * Vector3::z();
            Ok(ToolPose::new(
                [p.x * MM, p.y * MM, p.z * MM],
                [axis.x, axis.y, axis.z],
            ))
        };

        let mut events = run.events.iter().peekable();
        let mut building: Option<(Mode, SegmentBuilder)> = None;
        for pair in clock.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            while let Some(event) = events.next_if(|e| e.t <= a + 1e-9) {
                match event.event {
                    NcEvent::Tool(number) => {
                        change_tool(&mut tool_spans, number, window.t0 + event.t);
                        in_spindle = Some(number);
                    }
                    NcEvent::Spindle { .. } => spinning = true,
                    NcEvent::SpindleStop => spinning = false,
                    NcEvent::Coolant(_) | NcEvent::Function(_) | NcEvent::End => {}
                }
            }
            let middle = (a + b) / 2.0;
            let mode = in_spindle.map(|tool| Mode {
                line: run.line_at(middle).unwrap_or(0),
                rapid: run.rapid_at(middle),
                tool,
                spinning,
            });
            // A change of state closes the segment under way, cut in the
            // state it was built in.
            if building.as_ref().map(|(m, _)| m) != mode.as_ref() {
                if let Some((was, mut builder)) = building.take() {
                    if let Some(segment) = builder.flush() {
                        let (tool, _) = &tools[&was.tool];
                        pass.cut(
                            tool,
                            &segment,
                            &window.program,
                            &was,
                            window.t0 + a,
                            fixture_set.as_mut(),
                        )?;
                    }
                }
            }
            let Some(mode) = mode else { continue };
            if let std::collections::btree_map::Entry::Vacant(entry) = tools.entry(mode.tool) {
                match control.tools.get(&mode.tool).and_then(|t| t.body.as_ref()) {
                    Some(body) => {
                        entry.insert((cutter(body), body.length));
                    }
                    None => {
                        if !unbodied.contains(&mode.tool) {
                            unbodied.push(mode.tool);
                        }
                        continue;
                    }
                }
            }
            let (tool, length) = &tools[&mode.tool];
            if building.is_none() {
                let mut builder =
                    SegmentBuilder::new(5.0, 2.0).with_max_deviation(options.max_deviation * MM);
                builder.push(tip(a, *length)?);
                building = Some((mode.clone(), builder));
            }
            let (_, builder) = building.as_mut().expect("opened above");
            if let Some(segment) = builder.push(tip(b, *length)?) {
                pass.cut(
                    tool,
                    &segment,
                    &window.program,
                    &mode,
                    window.t0 + b,
                    fixture_set.as_mut(),
                )?;
            }
        }
        if let Some((was, mut builder)) = building.take() {
            if let Some(segment) = builder.flush() {
                let (tool, _) = &tools[&was.tool];
                pass.cut(
                    tool,
                    &segment,
                    &window.program,
                    &was,
                    window.t0 + duration,
                    fixture_set.as_mut(),
                )?;
            }
        }
        // Whatever the program switched after its last move still stands.
        for event in events.take_while(|e| e.t <= duration + 1e-9) {
            match event.event {
                NcEvent::Tool(number) => {
                    change_tool(&mut tool_spans, number, window.t0 + event.t);
                    in_spindle = Some(number);
                }
                NcEvent::Spindle { .. } => spinning = true,
                NcEvent::SpindleStop => spinning = false,
                NcEvent::Coolant(_) | NcEvent::Function(_) | NcEvent::End => {}
            }
        }
        // The part as the program leaves it is a state of its own.
        if pass.dirty {
            pass.snapshot(window.t0 + duration)?;
        }
    }

    let remaining_volume = pass.engine.total_volume() / MM3;
    let mesh = if pass.engine.parts().is_empty() {
        MeshData::new(Vec::new(), Vec::new())
    } else {
        let (mesh, kinds) = pass
            .engine
            .merged_mesh_with_kinds()
            .map_err(|e| fail(e.to_string()))?;
        colored_mesh(&mesh, &kinds, options.stock_color, options.cut_color)
    };
    let design = design
        .map(|(model, tolerance)| {
            let model = TriMesh {
                positions: model
                    .vertices
                    .iter()
                    .map(|v| [v[0] * MM, v[1] * MM, v[2] * MM])
                    .collect(),
                triangles: model.indices.clone(),
            };
            let compared = pass
                .engine
                .compare_with_design(&model, tolerance * MM)
                .map_err(|e| fail(format!("design model: {e}")))?;
            let meters = |mesh: &TriMesh| {
                MeshData::new(
                    mesh.positions
                        .iter()
                        .map(|p| [p[0] / MM, p[1] / MM, p[2] / MM])
                        .collect(),
                    mesh.triangles.clone(),
                )
            };
            let bounds = |b: Option<cutrail::Aabb>| {
                b.map(|b| {
                    (
                        [b.min[0] / MM, b.min[1] / MM, b.min[2] / MM],
                        [b.max[0] / MM, b.max[1] / MM, b.max[2] / MM],
                    )
                })
            };
            Ok(DesignCheck {
                tolerance,
                excess_volume: compared.excess_volume / MM3,
                gouge_volume: compared.gouge_volume / MM3,
                excess_bounds: bounds(compared.excess_bounds),
                gouge_bounds: bounds(compared.gouge_bounds),
                excess: meters(&compared.excess),
                gouge: meters(&compared.gouge),
            })
        })
        .transpose()?;
    Ok(Machining {
        carve: StockCarve {
            mesh,
            pose: obstacle.pose,
            chord_tolerance: options.chord_tolerance,
            initial_volume,
            removed_volume: initial_volume - remaining_volume,
            remaining_volume,
        },
        raw,
        stages: pass.stages,
        tools: tool_spans
            .into_iter()
            .map(|span| ToolSpan {
                to: span.to.min(timeline.duration),
                ..span
            })
            .collect(),
        changes,
        issues: pass.issues,
        lines: pass.lines,
        cut_span: pass.cut_span,
        unbodied,
        design,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nc::{NcMachine, NcProgram, NcTool};
    use botrail_model::RobotModel;
    use std::sync::Arc;

    const MACHINE: &str = "axes";

    /// A flat end mill 10 mm across with 20 mm of flutes in a 40 mm
    /// holder, `length` from gauge line to tip.
    fn end_mill(length: f64) -> NcToolBody {
        NcToolBody {
            cutter: NcCutter::Flat,
            diameter: 0.010,
            flute_length: 0.020,
            length,
            corner_radius: None,
            shank_diameter: None,
            holder: Some([0.040, 0.050]),
        }
    }

    /// The test axes at home with a 60 x 40 x 60 mm block on the middle
    /// of the table, work zero on the middle of its top, and tool 1
    /// entered as 120 mm long — carrying `body`.
    fn cell(body: NcToolBody, program: &str) -> (Scene, usize) {
        let model = Arc::new(RobotModel::from_urdf_str(crate::testdata::AXES_URDF).unwrap());
        let spindle = model.link_index("head").unwrap();
        let mut scene = Scene::new(model);
        let name = scene.robots()[0].name.clone();
        assert_eq!(name, MACHINE);
        scene
            .define_nc_machine(NcMachine {
                robot: name,
                axes: ["x".into(), "y".into(), "z".into()],
                max_feed: 0.2,
                acceleration: [5.0; 3],
                junction_deviation: 5e-5,
                tool_change_s: 1.0,
                spindle_max_rpm: Some(8000.0),
                work_offsets: Default::default(),
                tools: Default::default(),
                m_codes: Default::default(),
                changer: None,
            })
            .unwrap();
        // The table's top is 340 mm up at (-150, -100) with the axes at
        // home, the nose 740 mm up over the origin: the block's top is
        // 340 mm under the nose there.
        scene
            .add_obstacle(
                "block",
                Geometry::Box {
                    size: Vector3::new(0.06, 0.04, 0.06),
                },
                Isometry3::translation(-0.15, -0.10, 0.37),
            )
            .unwrap();
        scene
            .set_work_offset(MACHINE, "G54", Some([-0.15, -0.10, -0.34]))
            .unwrap();
        scene
            .set_nc_tool(
                MACHINE,
                1,
                Some(NcTool {
                    length: 0.120,
                    diameter: Some(0.010),
                    body: Some(body),
                }),
            )
            .unwrap();
        scene.add_nc_program(NcProgram {
            name: "op".into(),
            text: program.into(),
        });
        (scene, spindle)
    }

    /// A slot 5 mm deep along X, through the whole block.
    const SLOT: &str = "G90 G21 G53 G0 Z0.\nT1 M6\nS3000 M3\nG54 G0 X-40. Y0.\nG43 Z5. H1\n\
                        G1 Z-5. F300.\nX40. F600.\nG0 Z5.\nM5\nG53 G0 Z0.\nM30\n";

    fn machined(
        scene: &Scene,
        spindle: usize,
        fixtures: &[String],
        design: Option<(&MeshData, f64)>,
    ) -> Machining {
        let mut riders = vec!["block".to_string()];
        riders.extend(fixtures.iter().cloned());
        let timeline = scene
            .nc_dry_run(MACHINE, "op", "table", &riders, None)
            .unwrap();
        machine_stock(
            scene,
            &timeline,
            "block",
            0,
            spindle,
            fixtures,
            design,
            &MachiningOptions::default(),
        )
        .unwrap()
    }

    fn first_cut(result: &Machining) -> f64 {
        result.cut_span.unwrap().0
    }

    #[test]
    fn a_program_cuts_the_stock_that_rides_the_table() {
        let (scene, spindle) = cell(end_mill(0.120), SLOT);
        let result = machined(&scene, spindle, &[], None);
        // 60 x 10 x 5 mm of slot, on the one line that feeds across.
        let expected = 0.060 * 0.010 * 0.005;
        let off = (result.carve.removed_volume - expected).abs() / expected;
        assert!(off < 0.01, "removed {:.3e}", result.carve.removed_volume);
        assert_eq!(result.lines.len(), 1, "{:?}", result.lines);
        assert_eq!((result.lines[0].line, result.lines[0].tool), (7, 1));
        assert!((result.lines[0].volume - result.carve.removed_volume).abs() < 1e-15);
        assert!(result.issues.is_empty(), "{:?}", result.issues);
        assert!(result.unbodied.is_empty());
        // Tool 1 goes in at the change and stands to the end.
        assert_eq!(result.tools.len(), 1);
        assert_eq!(result.tools[0].tool, 1);
        assert!(result.tools[0].from > 0.0 && result.tools[0].from < first_cut(&result));
        assert!(result.raw.indices.len() == 12 && result.raw.vertices.len() == 8);
        // The stages are the stock as it was cut, the last one the part.
        let (first, last) = result.cut_span.unwrap();
        assert!(first < last && !result.stages.is_empty());
        let end = result.stages.last().unwrap();
        assert!((end.removed_volume - result.carve.removed_volume).abs() < 1e-15);
        assert_eq!(end.mesh.indices.len(), result.carve.mesh.indices.len());
        // The slot is in the block's own frame: its floor 5 mm under the
        // top, whichever way the table carried the block under the tool.
        let floor = result
            .carve
            .mesh
            .vertices
            .iter()
            .filter(|v| v[1].abs() < 0.004)
            .map(|v| v[2])
            .fold(f64::INFINITY, f64::min);
        assert!((floor - (0.030 - 0.005)).abs() < 1e-6, "floor at {floor}");
    }

    #[test]
    fn a_tool_longer_than_its_table_entry_meets_the_stock_at_rapid_and_by_line() {
        // The table says 120 mm; the tool is 135. The control brings the
        // tip to "5 mm above the part" — 10 mm into it — at rapid, over
        // the middle of the block.
        let program = "G90 G21 G53 G0 Z0.\nT1 M6\nS3000 M3\nG54 G0 X0. Y0.\nG43 Z5. H1\n\
                       G1 Z-2. F300.\nG0 Z5.\nM5\nG53 G0 Z0.\nM30\n";
        let (scene, spindle) = cell(end_mill(0.135), program);
        let result = machined(&scene, spindle, &[], None);
        let crash = &result.issues[0];
        assert_eq!(
            (crash.kind, crash.line, crash.tool, crash.what.as_str()),
            (MachiningIssueKind::RapidIntoStock, 5, 1, "block")
        );
        let bore = std::f64::consts::PI * 0.005f64.powi(2);
        assert!((crash.volume - bore * 0.010).abs() / (bore * 0.010) < 0.01);
        // The feed that follows goes 17 mm deep instead of 2: still within
        // the flutes, so the holder is clear and nothing else is wrong.
        assert_eq!(result.issues.len(), 1, "{:?}", result.issues);
        assert!((result.carve.removed_volume - bore * 0.017).abs() / (bore * 0.017) < 0.01);

        // Entered as it is, the same program is clean.
        let (mut scene, spindle) = cell(end_mill(0.135), program);
        scene
            .set_nc_tool(
                MACHINE,
                1,
                Some(NcTool {
                    length: 0.135,
                    diameter: Some(0.010),
                    body: Some(end_mill(0.135)),
                }),
            )
            .unwrap();
        let result = machined(&scene, spindle, &[], None);
        assert!(result.issues.is_empty(), "{:?}", result.issues);
        assert!((result.carve.removed_volume - bore * 0.002).abs() / (bore * 0.002) < 0.01);
    }

    #[test]
    fn a_cut_deeper_than_the_flutes_puts_the_shank_in_the_stock() {
        // 25 mm down beside the block with 20 mm of flutes, then into it:
        // the top 5 mm of the block is in the shank's way. (Straight down
        // a shank as wide as its cutter only follows the hole.)
        let program = "G90 G21 G53 G0 Z0.\nT1 M6\nS3000 M3\nG54 G0 X-40. Y0.\nG43 Z5. H1\n\
                       G1 Z-25. F300.\nX0. F600.\nG0 Z5.\nM5\nG53 G0 Z0.\nM30\n";
        let (scene, spindle) = cell(end_mill(0.120), program);
        let result = machined(&scene, spindle, &[], None);
        // And what the shank pushed through instead of cutting is still
        // there when the cutter is pulled straight up out of it, at rapid.
        let kinds: Vec<_> = result.issues.iter().map(|i| (i.kind, i.line)).collect();
        assert_eq!(
            kinds,
            [
                (MachiningIssueKind::HolderIntoStock, 7),
                (MachiningIssueKind::RapidIntoStock, 8)
            ]
        );
        // Plunged into the block instead, the same depth is only a hole.
        let plunge = program
            .replace("X-40. Y0.", "X0. Y0.")
            .replace("X0. F600.\n", "");
        let (scene, spindle) = cell(end_mill(0.120), &plunge);
        assert!(machined(&scene, spindle, &[], None).issues.is_empty());
    }

    #[test]
    fn a_stopped_spindle_and_a_fixture_in_the_way_are_named() {
        // No M3 (a line fewer): the slot is ploughed, not cut.
        let stopped = SLOT.replace("S3000 M3\n", "");
        let (scene, spindle) = cell(end_mill(0.120), &stopped);
        let result = machined(&scene, spindle, &[], None);
        assert_eq!(
            result
                .issues
                .iter()
                .map(|i| (i.kind, i.line))
                .collect::<Vec<_>>(),
            [(MachiningIssueKind::SpindleStopped, 6)]
        );

        // A jaw standing 2 mm proud of the slot's floor at the far end.
        let (mut scene, spindle) = cell(end_mill(0.120), SLOT);
        scene
            .add_obstacle(
                "jaw",
                Geometry::Box {
                    size: Vector3::new(0.01, 0.04, 0.02),
                },
                Isometry3::translation(-0.15 + 0.036, -0.10, 0.40 - 0.005 + 0.002 - 0.01),
            )
            .unwrap();
        let fixtures = vec!["jaw".to_string()];
        let result = machined(&scene, spindle, &fixtures, None);
        let hit = &result.issues[0];
        assert_eq!(
            (hit.kind, hit.line, hit.what.as_str()),
            (MachiningIssueKind::IntoFixture, 7, "jaw")
        );
        assert_eq!(result.issues.len(), 1, "{:?}", result.issues);
        // The stock is cut as it would be without the jaw.
        let expected = 0.060 * 0.010 * 0.005;
        assert!((result.carve.removed_volume - expected).abs() / expected < 0.01);
    }

    #[test]
    fn the_part_is_measured_against_its_design() {
        let (scene, spindle) = cell(end_mill(0.120), SLOT);
        let block = solid_mesh(
            "block",
            &Geometry::Box {
                size: Vector3::new(0.06, 0.04, 0.06),
            },
            1e-5,
        )
        .unwrap();
        // Designed as the uncut block, the slot is a gouge of its volume …
        let uncut = MeshData::new(block.0.clone(), block.1.clone());
        let check = machined(&scene, spindle, &[], Some((&uncut, 0.0)))
            .design
            .unwrap();
        let slot = 0.060 * 0.010 * 0.005;
        assert!((check.gouge_volume - slot).abs() / slot < 0.01);
        assert!(check.excess_volume < 1e-12 && check.excess_bounds.is_none());
        let (low, high) = check.gouge_bounds.unwrap();
        assert!((high[2] - 0.030).abs() < 1e-6 && (low[2] - 0.025).abs() < 1e-6);
        // … and designed 5 mm shorter all over, what stands either side
        // of the slot is left over.
        let faced = MeshData::new(
            block
                .0
                .iter()
                .map(|v| [v[0], v[1], v[2].min(0.025)])
                .collect(),
            block.1,
        );
        let check = machined(&scene, spindle, &[], Some((&faced, 0.0)))
            .design
            .unwrap();
        let rims = 0.060 * 0.030 * 0.005;
        assert!((check.excess_volume - rims).abs() / rims < 0.01);
        assert!(check.gouge_volume < 1e-12);
        // Within a tolerance wider than the step, nothing is left to say.
        let check = machined(&scene, spindle, &[], Some((&faced, 0.006)))
            .design
            .unwrap();
        assert!(check.excess_volume < 1e-12 && check.gouge_volume < 1e-12);
    }

    #[test]
    fn a_tool_without_a_body_cuts_nothing_and_is_named() {
        let (mut scene, spindle) = cell(end_mill(0.120), SLOT);
        scene
            .set_nc_tool(
                MACHINE,
                1,
                Some(NcTool {
                    length: 0.120,
                    diameter: Some(0.010),
                    body: None,
                }),
            )
            .unwrap();
        let result = machined(&scene, spindle, &[], None);
        assert_eq!(result.unbodied, [1]);
        assert!(result.carve.removed_volume == 0.0 && result.stages.is_empty());
    }
}
