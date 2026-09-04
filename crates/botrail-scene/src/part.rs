//! Part identity — what a resident *is* commercially — and the bill of
//! materials derived from it.
//!
//! A [`Part`] carries no geometry and no behaviour: it names the thing (a
//! catalog reference, a maker, a model number, a category), counts it, and
//! holds whatever free attributes the user wants summed later (`mass_kg`,
//! `power_w`, a price). Geometry stays on the obstacle, behaviour on the
//! device, electrical identity on the I/O map. Parts exist so that a bill
//! of materials can be *derived* from the scene instead of authored beside
//! it, in the same spirit as the I/O points being derived from the
//! sequences: the user authors what is in the cell and where, and the
//! parts list falls out.
//!
//! Identity attaches to a resident by name — a robot, an obstacle or a
//! whole obstacle group (a USD subtree, a generated fence — everything
//! under `<target>/`), a sensor, a device, an I/O node — through
//! [`Scene::set_part`]. Catalog robots and tools need no authoring at all:
//! their identity is read off the [`RobotSource`](botrail_model::RobotSource)
//! provenance record.
//!
//! [`Scene::bom`] lists every piece of *equipment* the scene holds —
//! robots and their tools, conveyors, axes, vehicles, sensors, controller
//! boxes — whether or not it has been identified (an unidentified line is
//! the to-do list of a purchasing sheet), plus every obstacle or group a
//! part was pinned to. Bare obstacles are geometry (scenery, stock, a
//! workpiece pool) and are not listed unless the user says they are parts.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use serde::{Deserialize, Serialize};

use crate::iomap::IoNodeKind;
use crate::seq::{DeviceKind, SensorKind};
use crate::{Scene, SceneError};

/// A catalog package reference: the id as resolved plus, when known, the
/// dataset revision it was fetched at. `revision: None` is a hand-written
/// reference that has not been pinned (a part named by its catalog id
/// without going through `Robot.from_catalog`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct CatalogRef {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub revision: Option<String>,
}

impl CatalogRef {
    /// `id@revision`, or the bare id when unpinned — the form BOM tables
    /// and generated scripts use.
    pub fn display(&self) -> String {
        match &self.revision {
            Some(revision) => format!("{}@{}", self.id, revision),
            None => self.id.clone(),
        }
    }

    /// Parses `id` or `id@revision`.
    pub fn parse(text: &str) -> CatalogRef {
        match text.split_once('@') {
            Some((id, revision)) if !revision.is_empty() => CatalogRef {
                id: id.to_string(),
                revision: Some(revision.to_string()),
            },
            _ => CatalogRef {
                id: text.trim_end_matches('@').to_string(),
                revision: None,
            },
        }
    }
}

/// A free part attribute: a number (summable — `mass_kg`, `power_w`, a
/// price) or a text (`ip_rating`, a note). Untagged on the wire, so a
/// project file reads `{"mass_kg": 45, "finish": "RAL 7035"}`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum PartAttr {
    Number(f64),
    Text(String),
}

impl PartAttr {
    pub fn as_number(&self) -> Option<f64> {
        match self {
            PartAttr::Number(n) => Some(*n),
            PartAttr::Text(_) => None,
        }
    }

    fn cell(&self) -> String {
        match self {
            PartAttr::Number(n) => trim_float(*n),
            PartAttr::Text(t) => t.clone(),
        }
    }
}

/// What a resident *is*: identity and count, nothing else. Every field is
/// optional so a part can be as thin as `model="GVL-1200"` and still land
/// on the BOM; the catalog reference is what `Robot.from_catalog` fills
/// in by itself.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Part {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub catalog: Option<CatalogRef>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    /// Model / part number.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    /// BOM category (`conveyor`, `sensor.photoelectric`, `structure.fence`,
    /// ...). Overrides the category the resident's kind implies.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub category: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// How many of them this target stands for. One resident is one part
    /// by default; a group generated as twelve fence panels says 12.
    #[serde(default = "one")]
    pub qty: u32,
    /// Free attributes, summed by [`Bom::total`] when numeric. botrail
    /// never validates them.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub attributes: BTreeMap<String, PartAttr>,
}

fn one() -> u32 {
    1
}

impl Default for Part {
    fn default() -> Self {
        Part {
            catalog: None,
            manufacturer: None,
            model: None,
            category: None,
            description: None,
            qty: 1,
            attributes: BTreeMap::new(),
        }
    }
}

impl Part {
    /// True when the part names nothing that identifies a product — no
    /// catalog reference, maker or model.
    pub fn is_unidentified(&self) -> bool {
        self.catalog.is_none() && self.manufacturer.is_none() && self.model.is_none()
    }
}

/// Which kind of resident a part entry is pinned to. Kinds have separate
/// name spaces (a conveyor device and its belt slab may both be `belt`),
/// so the kind is part of the key.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum PartTargetKind {
    Robot,
    Obstacle,
    /// Every obstacle under `<target>/` — an imported subtree, a generated
    /// assembly. Counted as one line.
    Group,
    Sensor,
    Device,
    /// A camera fixture (the purchasable `sensor.camera` article).
    Camera,
    /// A LiDAR scanner (the purchasable `sensor.lidar` article).
    Lidar,
    IoNode,
    /// An end-effector in a robot's tool stack, by the name its BOM row
    /// carries (`<robot>/tool`, `<robot>/tool2`, `<robot>/tool/tool3` for
    /// a tool that is itself a stack). A catalog tool brings its own
    /// identity; a made one — a bracket welded on from a URDF string —
    /// gets it pinned here.
    Tool,
}

impl PartTargetKind {
    pub fn as_str(self) -> &'static str {
        match self {
            PartTargetKind::Robot => "robot",
            PartTargetKind::Obstacle => "obstacle",
            PartTargetKind::Group => "group",
            PartTargetKind::Sensor => "sensor",
            PartTargetKind::Device => "device",
            PartTargetKind::Camera => "camera",
            PartTargetKind::Lidar => "lidar",
            PartTargetKind::IoNode => "io_node",
            PartTargetKind::Tool => "tool",
        }
    }

    pub fn parse(text: &str) -> Option<PartTargetKind> {
        Some(match text {
            "robot" => PartTargetKind::Robot,
            "obstacle" => PartTargetKind::Obstacle,
            "group" => PartTargetKind::Group,
            "sensor" => PartTargetKind::Sensor,
            "device" => PartTargetKind::Device,
            "camera" => PartTargetKind::Camera,
            "lidar" => PartTargetKind::Lidar,
            "io_node" => PartTargetKind::IoNode,
            "tool" => PartTargetKind::Tool,
            _ => return None,
        })
    }
}

/// One authored part pinning: `part` describes the resident (or group)
/// `target` of kind `kind`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct PartEntry {
    pub target: String,
    pub kind: PartTargetKind,
    pub part: Part,
}

/// What makes two BOM lines one product: category, catalog reference,
/// maker, model.
type MergeKey = (String, Option<String>, Option<String>, Option<String>);

/// One line of the bill of materials: identical products merged, their
/// resident names kept for traceability.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BomRow {
    /// Never empty: the part's own category or the one its resident kind
    /// implies (`manipulator`, `tool`, `conveyor`, `sensor.photoelectric`,
    /// `plc`, ..., `part` for a bare identified obstacle).
    pub category: String,
    /// The residents (or groups) this line stands for, in scene order.
    pub names: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub catalog: Option<CatalogRef>,
    pub qty: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub attributes: BTreeMap<String, PartAttr>,
}

impl BomRow {
    fn from_part(category: &str, name: &str, part: &Part) -> BomRow {
        BomRow {
            category: part
                .category
                .clone()
                .unwrap_or_else(|| category.to_string()),
            names: vec![name.to_string()],
            manufacturer: part.manufacturer.clone(),
            model: part.model.clone(),
            catalog: part.catalog.clone(),
            qty: part.qty,
            description: part.description.clone(),
            attributes: part.attributes.clone(),
        }
    }

    /// Lays an authored part over a derived line: every field the part
    /// states wins, its attributes are merged in over the derived ones,
    /// and its quantity replaces the derived one.
    fn apply(&mut self, part: &Part) {
        if let Some(category) = &part.category {
            self.category = category.clone();
        }
        if part.catalog.is_some() {
            self.catalog = part.catalog.clone();
        }
        if part.manufacturer.is_some() {
            self.manufacturer = part.manufacturer.clone();
        }
        if part.model.is_some() {
            self.model = part.model.clone();
        }
        if part.description.is_some() {
            self.description = part.description.clone();
        }
        self.qty = part.qty;
        for (key, value) in &part.attributes {
            self.attributes.insert(key.clone(), value.clone());
        }
    }

    /// The identity two lines must share to be one product. `None` for an
    /// unidentified line — those never merge, each stays under its own
    /// resident name so the sheet says which one still needs a number.
    fn merge_key(&self) -> Option<MergeKey> {
        if self.catalog.is_none() && self.manufacturer.is_none() && self.model.is_none() {
            return None;
        }
        Some((
            self.category.clone(),
            self.catalog.as_ref().map(CatalogRef::display),
            self.manufacturer.clone(),
            self.model.clone(),
        ))
    }

    /// True when nothing identifies the product yet.
    pub fn is_unidentified(&self) -> bool {
        self.merge_key().is_none()
    }
}

/// The bill of materials: one row per distinct product, in scene order
/// (robots and tools, then devices, sensors, I/O nodes, then identified
/// obstacles and groups in the order their parts were pinned).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Bom {
    pub rows: Vec<BomRow>,
}

impl Bom {
    /// Σ qty × attribute over the rows that carry `key` as a number, or
    /// `None` when no row does — a missing figure must not read as zero.
    pub fn total(&self, key: &str) -> Option<f64> {
        let mut sum = None;
        for row in &self.rows {
            if let Some(value) = row.attributes.get(key).and_then(PartAttr::as_number) {
                *sum.get_or_insert(0.0) += value * f64::from(row.qty);
            }
        }
        sum
    }

    /// Every attribute key any row carries, sorted — the extra columns of
    /// the tables.
    pub fn attribute_keys(&self) -> Vec<String> {
        let mut keys: Vec<String> = self
            .rows
            .iter()
            .flat_map(|row| row.attributes.keys().cloned())
            .collect();
        keys.sort();
        keys.dedup();
        keys
    }

    /// Rows whose product is still unidentified — the to-do list.
    pub fn unidentified(&self) -> Vec<&BomRow> {
        self.rows.iter().filter(|r| r.is_unidentified()).collect()
    }

    fn header(&self) -> Vec<String> {
        let mut columns: Vec<String> = [
            "category",
            "manufacturer",
            "model",
            "catalog",
            "qty",
            "description",
            "names",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        columns.extend(self.attribute_keys());
        columns
    }

    fn cells(&self, row: &BomRow) -> Vec<String> {
        let mut cells = vec![
            row.category.clone(),
            row.manufacturer.clone().unwrap_or_default(),
            row.model.clone().unwrap_or_default(),
            row.catalog
                .as_ref()
                .map(CatalogRef::display)
                .unwrap_or_default(),
            row.qty.to_string(),
            row.description.clone().unwrap_or_default(),
            row.names.join("; "),
        ];
        for key in self.attribute_keys() {
            cells.push(
                row.attributes
                    .get(&key)
                    .map(PartAttr::cell)
                    .unwrap_or_default(),
            );
        }
        cells
    }

    /// RFC 4180 CSV: header row, one line per product, attribute columns
    /// after the fixed ones.
    pub fn to_csv(&self) -> String {
        let mut out = String::new();
        out.push_str(
            &self
                .header()
                .iter()
                .map(|c| csv_cell(c))
                .collect::<Vec<_>>()
                .join(","),
        );
        out.push('\n');
        for row in &self.rows {
            out.push_str(
                &self
                    .cells(row)
                    .iter()
                    .map(|c| csv_cell(c))
                    .collect::<Vec<_>>()
                    .join(","),
            );
            out.push('\n');
        }
        out
    }

    /// A Markdown table with a numbered first column, followed by the
    /// numeric totals when any row carries one.
    pub fn to_markdown(&self) -> String {
        let mut out = String::new();
        let header = self.header();
        out.push_str("| # | ");
        out.push_str(&header.join(" | "));
        out.push_str(" |\n|---|");
        for _ in &header {
            out.push_str("---|");
        }
        out.push('\n');
        for (i, row) in self.rows.iter().enumerate() {
            let _ = write!(out, "| {} | ", i + 1);
            out.push_str(
                &self
                    .cells(row)
                    .iter()
                    .map(|c| md_cell(c))
                    .collect::<Vec<_>>()
                    .join(" | "),
            );
            out.push_str(" |\n");
        }
        let totals: Vec<String> = self
            .attribute_keys()
            .into_iter()
            .filter_map(|key| {
                self.total(&key)
                    .map(|t| format!("{key} = {}", trim_float(t)))
            })
            .collect();
        if !totals.is_empty() {
            let _ = write!(out, "\nTotals: {}\n", totals.join(", "));
        }
        out
    }

    /// `{"rows": [...], "totals": {...}}` — the machine-readable form.
    pub fn to_json(&self) -> String {
        let totals: BTreeMap<String, f64> = self
            .attribute_keys()
            .into_iter()
            .filter_map(|key| self.total(&key).map(|t| (key, t)))
            .collect();
        let doc = serde_json::json!({ "rows": self.rows, "totals": totals });
        serde_json::to_string_pretty(&doc).expect("BOM serializes")
    }
}

fn csv_cell(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_string()
    }
}

fn md_cell(text: &str) -> String {
    text.replace('|', "\\|").replace('\n', " ")
}

/// A number the way a table wants it: `45`, `2.5`, `855`, never `45.0`
/// or `2.5000000001`.
fn trim_float(value: f64) -> String {
    if value.fract() == 0.0 && value.abs() < 1e15 {
        format!("{}", value as i64)
    } else {
        let s = format!("{value:.6}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

/// Where a scene name resolves for part pinning.
fn resolve_hits(scene: &Scene, target: &str) -> Vec<PartTargetKind> {
    let mut hits = Vec::new();
    if scene.robot_index(target).is_some() {
        hits.push(PartTargetKind::Robot);
    }
    if scene.devices().iter().any(|d| d.name == target) {
        hits.push(PartTargetKind::Device);
    }
    if scene.sensors().iter().any(|s| s.name == target) {
        hits.push(PartTargetKind::Sensor);
    }
    if scene.cameras().iter().any(|c| c.name == target) {
        hits.push(PartTargetKind::Camera);
    }
    if scene.lidars().iter().any(|l| l.name == target) {
        hits.push(PartTargetKind::Lidar);
    }
    if scene.io_map().nodes.iter().any(|n| n.name == target) {
        hits.push(PartTargetKind::IoNode);
    }
    if tool_row_names(scene).iter().any(|n| n == target) {
        hits.push(PartTargetKind::Tool);
    }
    let prefix = format!("{target}/");
    let exact = scene.obstacles().iter().any(|o| o.name == target);
    let group = scene
        .obstacles()
        .iter()
        .any(|o| o.name.starts_with(&prefix));
    // A name that is both an obstacle and the parent of others is the
    // whole assembly — the exact prim is one piece of it.
    match (exact, group) {
        (_, true) => hits.push(PartTargetKind::Group),
        (true, false) => hits.push(PartTargetKind::Obstacle),
        (false, false) => {}
    }
    hits
}

/// The names the BOM gives the tools welded onto every robot — what a
/// `PartTargetKind::Tool` part is pinned to.
fn tool_row_names(scene: &Scene) -> Vec<String> {
    let mut names = Vec::new();
    for robot in &scene.robots {
        let mut rows = Vec::new();
        let mut tools = 0;
        robot_lines(
            &robot.model.source,
            &robot.name,
            "robot",
            &mut tools,
            &mut rows,
        );
        names.extend(rows.into_iter().skip(1).flat_map(|row| row.names));
    }
    names
}

fn target_exists(scene: &Scene, target: &str, kind: PartTargetKind) -> bool {
    match kind {
        PartTargetKind::Robot => scene.robot_index(target).is_some(),
        PartTargetKind::Tool => tool_row_names(scene).iter().any(|n| n == target),
        PartTargetKind::Device => scene.devices().iter().any(|d| d.name == target),
        PartTargetKind::Sensor => scene.sensors().iter().any(|s| s.name == target),
        PartTargetKind::Camera => scene.cameras().iter().any(|c| c.name == target),
        PartTargetKind::Lidar => scene.lidars().iter().any(|l| l.name == target),
        PartTargetKind::IoNode => scene.io_map().nodes.iter().any(|n| n.name == target),
        PartTargetKind::Obstacle => scene.obstacles().iter().any(|o| o.name == target),
        PartTargetKind::Group => {
            let prefix = format!("{target}/");
            scene
                .obstacles()
                .iter()
                .any(|o| o.name.starts_with(&prefix))
        }
    }
}

/// The category a resident kind implies when its part states none. The
/// names follow the catalog's `category` vocabulary where one exists
/// (`manipulator`, `plc`, `io.remote`, `sensor.photoelectric`, ...).
fn device_category(kind: &DeviceKind) -> Option<&'static str> {
    match kind {
        DeviceKind::Conveyor { .. } => Some("conveyor"),
        DeviceKind::LinearAxis { .. } => Some("axis.linear"),
        DeviceKind::Vehicle { .. } => Some("vehicle"),
        DeviceKind::Lift { .. } => Some("lift"),
        // A magazine and a return chute model an endless line; they are
        // not equipment anyone buys.
        DeviceKind::Source { .. } | DeviceKind::Sink { .. } => None,
    }
}

fn sensor_category(kind: &SensorKind) -> Option<&'static str> {
    match kind {
        SensorKind::Zone { .. } => Some("sensor.area"),
        SensorKind::Beam { .. } => Some("sensor.photoelectric"),
        // The purchasable article is the *camera* (`sensor.camera`); a
        // vision sensor is judgement bound to it, and a second BOM line
        // would double-count the hardware (design/design-camera.md 判断 10).
        SensorKind::Vision { .. } => None,
        // Same rule for the scanner: the article is the *lidar*
        // (`sensor.lidar`), a field is judgement bound to it
        // (design/design-lidar.md 判断 L1).
        SensorKind::Field { .. } => None,
    }
}

fn io_node_category(kind: &IoNodeKind) -> &'static str {
    match kind {
        IoNodeKind::Plc => "plc",
        IoNodeKind::SafetyPlc => "plc.safety",
        IoNodeKind::RemoteIo => "io.remote",
        IoNodeKind::RobotController { .. } => "robot_controller",
        IoNodeKind::Other { .. } => "other",
    }
}

/// The BOM lines a robot's provenance implies: a catalog package is a
/// fully identified line, a composite is its base plus its tool(s), a bare
/// URDF/USD robot is an unidentified line under the instance name.
fn robot_lines(
    source: &botrail_model::RobotSource,
    name: &str,
    role_category: &str,
    tool_counter: &mut usize,
    out: &mut Vec<BomRow>,
) {
    use botrail_model::RobotSource;
    match source {
        RobotSource::Catalog {
            id, revision, meta, ..
        } => {
            let attributes: BTreeMap<String, PartAttr> = meta
                .specs
                .iter()
                .map(|(k, v)| (k.clone(), PartAttr::Number(*v)))
                .collect();
            out.push(BomRow {
                category: meta
                    .category
                    .clone()
                    .unwrap_or_else(|| role_category.to_string()),
                names: vec![name.to_string()],
                manufacturer: meta.manufacturer.clone(),
                model: meta.product.clone(),
                catalog: Some(CatalogRef {
                    id: id.clone(),
                    revision: Some(revision.clone()),
                }),
                qty: 1,
                description: None,
                attributes,
            });
        }
        RobotSource::Composite { base, tool, .. } => {
            robot_lines(base, name, role_category, tool_counter, out);
            *tool_counter += 1;
            let tool_name = if *tool_counter == 1 {
                format!("{name}/tool")
            } else {
                format!("{name}/tool{tool_counter}")
            };
            robot_lines(tool, &tool_name, "tool", tool_counter, out);
        }
        RobotSource::UrdfXml(_) | RobotSource::Usd { .. } => out.push(BomRow {
            category: role_category.to_string(),
            names: vec![name.to_string()],
            manufacturer: None,
            model: None,
            catalog: None,
            qty: 1,
            description: None,
            attributes: BTreeMap::new(),
        }),
    }
}

/// Merges identical products, keeping first-seen order.
fn merge_rows(lines: Vec<BomRow>) -> Vec<BomRow> {
    let mut rows: Vec<BomRow> = Vec::new();
    for line in lines {
        let Some(key) = line.merge_key() else {
            rows.push(line);
            continue;
        };
        match rows
            .iter_mut()
            .find(|r| r.merge_key().as_ref() == Some(&key))
        {
            Some(row) => {
                row.qty += line.qty;
                row.names.extend(line.names);
                if row.description.is_none() {
                    row.description = line.description;
                }
                for (k, v) in line.attributes {
                    row.attributes.entry(k).or_insert(v);
                }
            }
            None => rows.push(line),
        }
    }
    rows
}

impl Scene {
    // ------------------------------------------------------------ authoring

    /// The authored part pinnings, in authoring order.
    pub fn parts(&self) -> &[PartEntry] {
        &self.parts
    }

    /// The part pinned to `target` (any kind), if one is.
    pub fn part(&self, target: &str) -> Option<&PartEntry> {
        self.parts.iter().find(|p| p.target == target)
    }

    /// Pins a part to a resident or group by name and returns the kind it
    /// resolved to. Without `kind` the name is looked up in every name
    /// space (robot, device, sensor, I/O node, obstacle, group of
    /// obstacles under `<name>/`); a name that lives in several is an
    /// error asking for the kind. Re-pinning the same target replaces the
    /// part.
    pub fn set_part(
        &mut self,
        target: &str,
        kind: Option<PartTargetKind>,
        part: Part,
    ) -> Result<PartTargetKind, SceneError> {
        let kind = match kind {
            Some(kind) => {
                if !target_exists(self, target, kind) {
                    return Err(SceneError::BadPart(format!(
                        "no {} named `{target}`",
                        kind.as_str()
                    )));
                }
                kind
            }
            None => {
                let hits = resolve_hits(self, target);
                match hits.as_slice() {
                    [one] => *one,
                    [] => {
                        return Err(SceneError::BadPart(format!(
                            "`{target}` is not a robot, tool, device, sensor, camera, lidar, \
                             I/O node, obstacle or obstacle group in this scene"
                        )))
                    }
                    many => {
                        return Err(SceneError::BadPart(format!(
                            "`{target}` names several things ({}); pass kind=",
                            many.iter()
                                .map(|k| k.as_str())
                                .collect::<Vec<_>>()
                                .join(", ")
                        )))
                    }
                }
            }
        };
        if part.qty == 0 {
            return Err(SceneError::BadPart(format!(
                "part `{target}`: qty must be at least 1"
            )));
        }
        match self
            .parts
            .iter_mut()
            .find(|p| p.target == target && p.kind == kind)
        {
            Some(slot) => slot.part = part,
            None => self.parts.push(PartEntry {
                target: target.to_string(),
                kind,
                part,
            }),
        }
        Ok(kind)
    }

    /// Unpins the part(s) on `target`.
    pub fn remove_part(&mut self, target: &str) -> Result<(), SceneError> {
        let before = self.parts.len();
        self.parts.retain(|p| p.target != target);
        if self.parts.len() == before {
            return Err(SceneError::BadPart(format!("no part pinned to `{target}`")));
        }
        Ok(())
    }

    /// Replaces the whole pinning list (project load). Every entry must
    /// resolve.
    pub fn set_parts(&mut self, parts: Vec<PartEntry>) -> Result<(), SceneError> {
        for entry in &parts {
            if !target_exists(self, &entry.target, entry.kind) {
                return Err(SceneError::BadPart(format!(
                    "part `{}`: no {} of that name",
                    entry.target,
                    entry.kind.as_str()
                )));
            }
        }
        self.parts = parts;
        Ok(())
    }

    /// Drops pinnings whose target has left the scene. Called by the
    /// resident removals so a deleted conveyor does not linger on the BOM.
    pub(crate) fn prune_parts(&mut self) {
        let keep: Vec<bool> = self
            .parts
            .iter()
            .map(|p| target_exists(self, &p.target, p.kind))
            .collect();
        let mut it = keep.into_iter();
        self.parts.retain(|_| it.next().unwrap_or(false));
    }

    /// Follows a robot rename.
    pub(crate) fn rename_part_target(&mut self, kind: PartTargetKind, old: &str, new: &str) {
        let prefix = format!("{old}/");
        for entry in &mut self.parts {
            if entry.kind == kind && entry.target == old {
                entry.target = new.to_string();
            } else if kind == PartTargetKind::Robot
                && entry.kind == PartTargetKind::Tool
                && entry.target.starts_with(&prefix)
            {
                // The tools ride the robot's name: `arm/tool` follows `arm`.
                entry.target = format!("{new}/{}", &entry.target[prefix.len()..]);
            }
        }
    }

    // ------------------------------------------------------------------ BOM

    /// The bill of materials derived from the scene: robots and tools
    /// (identity from their catalog provenance), conveyors / axes /
    /// vehicles, sensors, I/O nodes — each once, identified or not — plus
    /// every obstacle or group a part is pinned to. Identical products
    /// merge into one row with the quantity summed.
    pub fn bom(&self) -> Bom {
        let mut lines: Vec<BomRow> = Vec::new();
        let explicit = |kind: PartTargetKind, name: &str| -> Option<&Part> {
            self.parts
                .iter()
                .find(|p| p.kind == kind && p.target == name)
                .map(|p| &p.part)
        };

        for robot in &self.robots {
            let mut robot_rows = Vec::new();
            let mut tools = 0;
            robot_lines(
                &robot.model.source,
                &robot.name,
                "robot",
                &mut tools,
                &mut robot_rows,
            );
            if let Some(part) = explicit(PartTargetKind::Robot, &robot.name) {
                if let Some(first) = robot_rows.first_mut() {
                    first.apply(part);
                }
            }
            // The tools after it, each by its row name: a pinned part is
            // the identity of a made tool, or the last word on a catalog one.
            for row in robot_rows.iter_mut().skip(1) {
                if let Some(part) = row
                    .names
                    .first()
                    .and_then(|n| explicit(PartTargetKind::Tool, n))
                {
                    row.apply(part);
                }
            }
            lines.extend(robot_rows);
        }
        for device in &self.devices {
            let part = explicit(PartTargetKind::Device, &device.name);
            // A vehicle whose machine is a robot *is* that robot: legs (a
            // gait mount), or the whole airframe (a rigid mount on a
            // vehicle with no body of its own — a UAV). One machine, listed
            // once on the robot's line — unless the device was pinned to a
            // part of its own. An AMR carrying an arm stays two rows: the
            // chassis has a body, so vehicle and rider are two products.
            let bodiless = matches!(&device.kind,
                DeviceKind::Vehicle { body, .. } if body.is_empty());
            let is_the_robot = self.robots.iter().any(|r| {
                r.mount
                    .as_ref()
                    .is_some_and(|m| m.device == device.name && (m.gait.is_some() || bodiless))
            });
            if is_the_robot && part.is_none() {
                continue;
            }
            let Some(category) = device_category(&device.kind).or(part.and(Some("device"))) else {
                continue;
            };
            let mut row = BomRow::from_part(category, &device.name, &Part::default());
            if let Some(part) = part {
                row.apply(part);
            }
            lines.push(row);
        }
        for sensor in &self.sensors {
            let Some(category) = sensor_category(&sensor.kind) else {
                continue;
            };
            let mut row = BomRow::from_part(category, &sensor.name, &Part::default());
            if let Some(part) = explicit(PartTargetKind::Sensor, &sensor.name) {
                row.apply(part);
            }
            lines.push(row);
        }
        for camera in &self.cameras {
            // The purchasable article: any vision sensors looking through
            // it are judgement, not hardware (they add no line of their
            // own — see `sensor_category`).
            let mut row = BomRow::from_part("sensor.camera", &camera.name, &Part::default());
            if let Some(part) = explicit(PartTargetKind::Camera, &camera.name) {
                row.apply(part);
            }
            lines.push(row);
        }
        for lidar in &self.lidars {
            // The purchasable article: any field sensors sweeping through
            // it are judgement, not hardware (design/design-lidar.md 判断
            // L1 — the camera rule, applied to scanners).
            let mut row = BomRow::from_part("sensor.lidar", &lidar.name, &Part::default());
            if let Some(part) = explicit(PartTargetKind::Lidar, &lidar.name) {
                row.apply(part);
            }
            lines.push(row);
        }
        for node in &self.io.nodes {
            let mut row =
                BomRow::from_part(io_node_category(&node.kind), &node.name, &Part::default());
            // The I/O map's own model column is the node's identity until a
            // part says otherwise.
            row.model = node.model.clone();
            if let IoNodeKind::Other { label } = &node.kind {
                row.description = Some(label.clone());
            }
            if let Some(part) = explicit(PartTargetKind::IoNode, &node.name) {
                row.apply(part);
            }
            lines.push(row);
        }
        for entry in &self.parts {
            if matches!(entry.kind, PartTargetKind::Obstacle | PartTargetKind::Group) {
                lines.push(BomRow::from_part("part", &entry.target, &entry.part));
            }
        }
        Bom {
            rows: merge_rows(lines),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::{Device, DeviceKind, Sensor, SensorKind, SensorWatch};
    use botrail_model::{Geometry, RobotModel};
    use nalgebra::{Isometry3, Point3, Vector3};
    use std::sync::Arc;

    const URDF: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="l1"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>"#;

    fn scene() -> Scene {
        Scene::new(Arc::new(RobotModel::from_urdf_str(URDF).unwrap()))
    }

    fn box_geometry() -> Geometry {
        Geometry::Box {
            size: Vector3::new(0.1, 0.1, 0.1),
        }
    }

    fn part(model: &str) -> Part {
        Part {
            model: Some(model.to_string()),
            ..Part::default()
        }
    }

    #[test]
    fn robot_without_catalog_is_an_unidentified_line() {
        let scene = scene();
        let bom = scene.bom();
        assert_eq!(bom.rows.len(), 1);
        assert_eq!(bom.rows[0].category, "robot");
        assert_eq!(bom.rows[0].names, vec!["arm".to_string()]);
        assert!(bom.rows[0].is_unidentified());
        assert_eq!(bom.unidentified().len(), 1);
    }

    #[test]
    fn set_part_resolves_kind_and_identifies_the_line() {
        let mut scene = scene();
        let mut p = part("M-20iD/25");
        p.manufacturer = Some("FANUC".into());
        p.attributes
            .insert("mass_kg".into(), PartAttr::Number(250.0));
        assert_eq!(
            scene.set_part("arm", None, p).unwrap(),
            PartTargetKind::Robot
        );
        let bom = scene.bom();
        assert_eq!(bom.rows[0].manufacturer.as_deref(), Some("FANUC"));
        assert_eq!(bom.rows[0].model.as_deref(), Some("M-20iD/25"));
        assert_eq!(bom.total("mass_kg"), Some(250.0));
        assert_eq!(bom.total("price"), None);
    }

    #[test]
    fn identical_products_merge_and_unidentified_do_not() {
        let mut scene = scene();
        for name in ["table_a", "table_b", "table_c"] {
            scene
                .add_obstacle(name, box_geometry(), Isometry3::identity())
                .unwrap();
        }
        // Two identified as the same table, one left bare (not a part at
        // all — bare obstacles are geometry).
        let mut p = part("HFS8-1200");
        p.attributes
            .insert("mass_kg".into(), PartAttr::Number(30.0));
        scene.set_part("table_a", None, p.clone()).unwrap();
        scene.set_part("table_b", None, p).unwrap();
        // Two unidentified sensors stay two lines.
        for name in ["eye_1", "eye_2"] {
            scene
                .upsert_sensor(Sensor {
                    name: name.into(),
                    kind: SensorKind::Beam {
                        from: Point3::origin(),
                        to: Point3::new(1.0, 0.0, 0.0),
                        radius: 0.01,
                    },
                    watch: SensorWatch::All,
                    mount: None,
                })
                .unwrap();
        }
        let bom = scene.bom();
        let tables: Vec<&BomRow> = bom.rows.iter().filter(|r| r.category == "part").collect();
        assert_eq!(tables.len(), 1);
        assert_eq!(tables[0].qty, 2);
        assert_eq!(
            tables[0].names,
            vec!["table_a".to_string(), "table_b".to_string()]
        );
        assert_eq!(bom.total("mass_kg"), Some(60.0));
        let eyes: Vec<&BomRow> = bom
            .rows
            .iter()
            .filter(|r| r.category == "sensor.photoelectric")
            .collect();
        assert_eq!(eyes.len(), 2);
    }

    #[test]
    fn group_prefix_is_one_line_and_ambiguity_needs_a_kind() {
        let mut scene = scene();
        for name in ["env/Pedestal/Column", "env/Pedestal/Plate"] {
            scene
                .add_obstacle(name, box_geometry(), Isometry3::identity())
                .unwrap();
        }
        assert_eq!(
            scene
                .set_part("env/Pedestal", None, part("PD-500"))
                .unwrap(),
            PartTargetKind::Group
        );
        // A device and an obstacle sharing a name is ambiguous...
        scene
            .add_obstacle("belt", box_geometry(), Isometry3::identity())
            .unwrap();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: Isometry3::identity(),
                zone_size: Vector3::new(1.0, 1.0, 1.0),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        assert!(matches!(
            scene.set_part("belt", None, part("GVL")),
            Err(SceneError::BadPart(_))
        ));
        // ...until the kind is named.
        scene
            .set_part("belt", Some(PartTargetKind::Device), part("GVL-1200"))
            .unwrap();
        let bom = scene.bom();
        let conveyor = bom.rows.iter().find(|r| r.category == "conveyor").unwrap();
        assert_eq!(conveyor.model.as_deref(), Some("GVL-1200"));
        assert_eq!(bom.rows.iter().filter(|r| r.category == "part").count(), 1);
        // Unknown names are rejected.
        assert!(scene.set_part("nothing", None, part("x")).is_err());
    }

    #[test]
    fn removing_the_resident_drops_its_part() {
        let mut scene = scene();
        scene
            .add_obstacle("g/a", box_geometry(), Isometry3::identity())
            .unwrap();
        scene.set_part("g", None, part("G-1")).unwrap();
        assert_eq!(scene.parts().len(), 1);
        scene.remove_obstacle("g/a").unwrap();
        assert!(scene.parts().is_empty());
        assert!(scene.remove_part("g").is_err());
    }

    #[test]
    fn tables_render() {
        let mut scene = scene();
        let mut p = part("A, \"quoted\"");
        p.attributes.insert("mass_kg".into(), PartAttr::Number(2.5));
        p.attributes
            .insert("note".into(), PartAttr::Text("x|y".into()));
        scene.set_part("arm", None, p).unwrap();
        let bom = scene.bom();
        let csv = bom.to_csv();
        assert!(csv.starts_with(
            "category,manufacturer,model,catalog,qty,description,names,mass_kg,note\n"
        ));
        assert!(csv.contains("\"A, \"\"quoted\"\"\""));
        let md = bom.to_markdown();
        assert!(md.contains("x\\|y"));
        assert!(md.contains("Totals: mass_kg = 2.5"));
        let json: serde_json::Value = serde_json::from_str(&bom.to_json()).unwrap();
        assert_eq!(json["totals"]["mass_kg"], 2.5);
        assert_eq!(json["rows"][0]["model"], "A, \"quoted\"");
    }

    #[test]
    fn catalog_ref_parses_pinned_and_bare() {
        assert_eq!(
            CatalogRef::parse("keyence/pz-g61n@abc"),
            CatalogRef {
                id: "keyence/pz-g61n".into(),
                revision: Some("abc".into())
            }
        );
        assert_eq!(
            CatalogRef::parse("keyence/pz-g61n"),
            CatalogRef {
                id: "keyence/pz-g61n".into(),
                revision: None
            }
        );
        assert_eq!(CatalogRef::parse("a@").display(), "a");
    }
}
