//! Declared mechanical interfaces. Values describe bare mating faces, not
//! the compatibility of a complete robot/tool stack. No geometric inference.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct CatalogSource {
    pub kind: String,
    pub url: String,
    #[serde(default, rename = "ref")]
    pub revision: Option<String>,
    #[serde(default)]
    pub fetched_at: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct CatalogOrder {
    #[serde(default)]
    pub part_number: Option<String>,
    #[serde(default = "each")]
    pub unit: String,
    #[serde(default)]
    pub includes: Vec<OrderInclude>,
    #[serde(default)]
    pub requires: Vec<OrderRequirement>,
    #[serde(default)]
    pub note: Option<String>,
}

fn each() -> String {
    "each".into()
}
fn one() -> u32 {
    1
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct OrderInclude {
    pub name: String,
    #[serde(default)]
    pub catalog: Option<String>,
    #[serde(default)]
    pub part_number: Option<String>,
    #[serde(default = "one")]
    pub qty: u32,
    #[serde(default)]
    pub note: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct OrderRequirement {
    #[serde(default)]
    pub category: Option<String>,
    #[serde(default)]
    pub catalog: Option<String>,
    #[serde(default)]
    pub part_number: Option<String>,
    #[serde(default = "one")]
    pub qty: u32,
    #[serde(default)]
    pub note: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountEvidence {
    /// Index into the package's existing `sources` array.
    pub source: usize,
    /// Drawing page, manual section, or other exact location in the source.
    pub section: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum InterfaceRole {
    Mount,
    Flange,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountPose {
    /// Flange-to-mount translation, metres. Not adapter thickness.
    pub position: [f64; 3],
    /// Unit quaternion, XYZW, in the model's declared frame convention.
    pub quaternion: [f64; 4],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountInterface {
    pub frame: String,
    pub role: InterfaceRole,
    /// Shared identifier of the bare mating interface. None = not known.
    #[serde(default)]
    pub interface_id: Option<String>,
    /// Explicit permitted transforms at a mount. None = not documented.
    #[serde(default)]
    pub allowed_poses: Option<Vec<MountPose>>,
    #[serde(default)]
    pub evidence: Vec<MountEvidence>,
    #[serde(default)]
    pub note: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geometry: Option<MountGeometry>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clearance: Option<MountClearance>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fasteners: Vec<MountFastener>,
    /// The installation documentation enumerates all required separate parts.
    #[serde(default)]
    pub requirements_complete: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountRequirement {
    pub id: String,
    /// The mount whose actual upstream attachment path must contain the part.
    pub frame: String,
    /// Index into existing `order.requires`. None = part not specified yet.
    #[serde(default)]
    pub order_requires: Option<usize>,
    #[serde(default)]
    pub evidence: Vec<MountEvidence>,
    pub note: String,
    /// An explicitly permitted alternative adapter, described by its two
    /// interfaces. Absent for product-specific/electronic couplings.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub interface_pair: Option<InterfacePair>,
    /// Individually sourced product alternatives, including OEM electronic
    /// couplings. These are exact catalog identities, not face equivalence.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub catalog_alternatives: Vec<MountAlternative>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountAlternative {
    pub catalog: String,
    #[serde(default)]
    pub evidence: Vec<MountEvidence>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct InterfacePair {
    pub mount: String,
    pub flange: String,
}

/// Explicit dimensional bounds. Equal bounds describe a nominal value only;
/// dimensional completeness must also be established by the source drawing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct DimensionRange {
    pub min: f64,
    pub max: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountThread {
    pub diameter_mm: f64,
    #[serde(default)]
    pub pitch_mm: Option<f64>,
    #[serde(default)]
    pub left_hand: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum HoleKind {
    Clearance,
    Threaded,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct FastenerRules {
    #[serde(default)]
    pub thread: Option<MountThread>,
    /// Optional manufacturer-prescribed screw length, under the head.
    #[serde(default)]
    pub length_mm: Option<DimensionRange>,
    #[serde(default)]
    pub head_standard: Option<String>,
    #[serde(default)]
    pub property_class: Option<String>,
    #[serde(default)]
    pub min_engagement_mm: Option<f64>,
    #[serde(default)]
    pub torque_nm: Option<DimensionRange>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountHole {
    pub id: String,
    /// XY coordinates in the mating plane of this link, millimetres.
    pub position_mm: [f64; 2],
    pub kind: HoleKind,
    #[serde(default)]
    pub position_tolerance_mm: Option<f64>,
    #[serde(default)]
    pub diameter_mm: Option<DimensionRange>,
    #[serde(default)]
    pub thread: Option<MountThread>,
    /// Maximum usable tip depth from the mating plane, including bottom allowance.
    #[serde(default)]
    pub depth_mm: Option<DimensionRange>,
    /// Unthreaded distance from the mating plane to the first usable receiver
    /// thread. Omitted = threads start at the plane (the original contract).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thread_start_mm: Option<DimensionRange>,
    /// Bearing surface to mating face, accounting for any counterbore.
    #[serde(default)]
    pub grip_mm: Option<DimensionRange>,
    #[serde(default)]
    pub fastener_rules: Option<FastenerRules>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum LocatorKind {
    Boss,
    Recess,
    Pin,
    Hole,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountLocator {
    pub id: String,
    pub kind: LocatorKind,
    pub position_mm: [f64; 2],
    #[serde(default)]
    pub position_tolerance_mm: Option<f64>,
    #[serde(default)]
    pub diameter_mm: Option<DimensionRange>,
    /// Projection height (boss/pin) or usable recess/hole depth.
    #[serde(default)]
    pub depth_mm: Option<DimensionRange>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountGeometry {
    /// Outward surface normal along local Z: exactly +1 or -1.
    pub normal_z: i8,
    #[serde(default)]
    pub frame_verified: bool,
    /// All mating features and their tolerances are covered by the drawing.
    #[serde(default)]
    pub complete: bool,
    #[serde(default)]
    pub holes: Vec<MountHole>,
    #[serde(default)]
    pub locators: Vec<MountLocator>,
    #[serde(default)]
    pub evidence: Vec<MountEvidence>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountFastener {
    /// Hole IDs on the clearance-hole side. One screw per listed hole.
    pub holes: Vec<String>,
    pub thread: MountThread,
    pub head_standard: String,
    pub property_class: String,
    pub length_mm: DimensionRange,
    /// Explicitly zero when no washer is used.
    pub washer_mm: DimensionRange,
    pub torque_nm: DimensionRange,
    #[serde(default)]
    pub evidence: Vec<MountEvidence>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountBox {
    pub id: String,
    pub center_mm: [f64; 3],
    pub size_mm: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountClearance {
    #[serde(default)]
    pub frame_verified: bool,
    /// Conservative solids and every required access space for this joint.
    #[serde(default)]
    pub complete: bool,
    #[serde(default)]
    pub solids: Vec<MountBox>,
    #[serde(default)]
    pub access: Vec<MountBox>,
    #[serde(default)]
    pub evidence: Vec<MountEvidence>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountingDocument {
    pub schema_version: String,
    pub revision: String,
    pub mounting: MountingSpec,
    #[serde(default)]
    pub order: Option<CatalogOrder>,
    #[serde(default)]
    pub sources: Vec<CatalogSource>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountingSpec {
    #[serde(default)]
    pub interfaces: Vec<MountInterface>,
    #[serde(default)]
    pub requirements: Vec<MountRequirement>,
    #[serde(default)]
    pub note: Option<String>,
}

impl MountingSpec {
    /// Validate declarations when loading packages or saved projects. Missing
    /// evidence is valid input (reviewed as unknown); broken references are not.
    pub fn validate(
        &self,
        sources: &[CatalogSource],
        order: Option<&CatalogOrder>,
    ) -> Result<(), String> {
        let evidence = |items: &[MountEvidence]| -> Result<(), String> {
            for e in items {
                if e.source >= sources.len() || e.section.trim().is_empty() {
                    return Err("mounting evidence needs a valid sources index and section".into());
                }
            }
            Ok(())
        };
        let mut frames = std::collections::HashSet::new();
        for face in &self.interfaces {
            if face.frame.trim().is_empty() || !frames.insert((&face.frame, face.role as u8)) {
                return Err("mounting interfaces need unique, nonempty frame/role pairs".into());
            }
            if face
                .interface_id
                .as_ref()
                .is_some_and(|s| s.trim().is_empty())
            {
                return Err("mounting interface_id cannot be empty".into());
            }
            evidence(&face.evidence)?;
            if let Some(g) = &face.geometry {
                if !matches!(g.normal_z, -1 | 1) {
                    return Err("mounting normal_z must be +1 or -1".into());
                }
                evidence(&g.evidence)?;
                let mut features = std::collections::HashSet::new();
                for h in &g.holes {
                    feature(
                        &h.id,
                        &h.position_mm,
                        h.position_tolerance_mm,
                        &mut features,
                    )?;
                    for r in [h.diameter_mm, h.depth_mm, h.grip_mm, h.thread_start_mm]
                        .into_iter()
                        .flatten()
                    {
                        r.validate()?;
                    }
                    if let Some(t) = &h.thread {
                        t.validate()?;
                    }
                    if h.kind == HoleKind::Clearance
                        && (h.thread.is_some() || h.thread_start_mm.is_some())
                    {
                        return Err("a clearance hole cannot declare a thread".into());
                    }
                    if let Some(rules) = &h.fastener_rules {
                        if let Some(thread) = &rules.thread {
                            thread.validate()?;
                        }
                        if let Some(v) = rules.min_engagement_mm {
                            positive(v)?;
                        }
                        if let Some(v) = rules.torque_nm {
                            v.validate()?;
                        }
                        if let Some(v) = rules.length_mm {
                            v.validate()?;
                            positive(v.min)?;
                        }
                        for s in [rules.head_standard.as_ref(), rules.property_class.as_ref()]
                            .into_iter()
                            .flatten()
                        {
                            nonempty(s)?;
                        }
                    }
                }
                for l in &g.locators {
                    feature(
                        &l.id,
                        &l.position_mm,
                        l.position_tolerance_mm,
                        &mut features,
                    )?;
                    for r in [l.diameter_mm, l.depth_mm].into_iter().flatten() {
                        r.validate()?;
                    }
                }
            }
            let mut selected = std::collections::HashSet::new();
            for f in &face.fasteners {
                f.thread.validate()?;
                f.length_mm.validate()?;
                f.washer_mm.validate()?;
                f.torque_nm.validate()?;
                positive(f.length_mm.min)?;
                nonempty(&f.head_standard)?;
                nonempty(&f.property_class)?;
                evidence(&f.evidence)?;
                if f.holes.is_empty() {
                    return Err("fastener holes must not be empty".into());
                }
                for id in &f.holes {
                    if !selected.insert(id)
                        || !face.geometry.as_ref().is_some_and(|g| {
                            g.holes
                                .iter()
                                .any(|h| &h.id == id && h.kind == HoleKind::Clearance)
                        })
                    {
                        return Err(
                            "fasteners must reference distinct declared clearance holes".into()
                        );
                    }
                }
            }
            if let Some(c) = &face.clearance {
                evidence(&c.evidence)?;
                if c.complete && c.solids.is_empty() {
                    return Err("complete clearance requires conservative solid envelopes".into());
                }
                let mut ids = std::collections::HashSet::new();
                for b in c.solids.iter().chain(&c.access) {
                    nonempty(&b.id)?;
                    if !ids.insert(&b.id) || !b.center_mm.iter().all(|v| v.is_finite()) {
                        return Err("clearance boxes need unique ids and finite centers".into());
                    }
                    for v in b.size_mm {
                        positive(v)?;
                    }
                }
            }
            if let Some(poses) = &face.allowed_poses {
                if face.role != InterfaceRole::Mount || poses.is_empty() {
                    return Err("allowed_poses must be a nonempty list on a mount interface".into());
                }
                for pose in poses {
                    let norm: f64 = pose.quaternion.iter().map(|x| x * x).sum();
                    if !pose
                        .position
                        .iter()
                        .chain(&pose.quaternion)
                        .all(|v| v.is_finite())
                        || (norm - 1.0).abs() > 1e-6
                    {
                        return Err(
                            "mounting pose needs finite metres and a unit XYZW quaternion".into(),
                        );
                    }
                }
            }
        }
        let mut ids = std::collections::HashSet::new();
        for r in &self.requirements {
            if r.id.trim().is_empty()
                || !ids.insert(&r.id)
                || r.note.trim().is_empty()
                || !self
                    .interfaces
                    .iter()
                    .any(|f| f.frame == r.frame && f.role == InterfaceRole::Mount)
            {
                return Err(
                    "mounting requirement needs a unique id, note and declared mount frame".into(),
                );
            }
            evidence(&r.evidence)?;
            let mut alternatives = std::collections::HashSet::new();
            for alternative in &r.catalog_alternatives {
                nonempty(&alternative.catalog)?;
                if !alternatives.insert(&alternative.catalog) {
                    return Err("mounting catalog alternatives must be unique".into());
                }
                evidence(&alternative.evidence)?;
            }
            if let Some(pair) = &r.interface_pair {
                nonempty(&pair.mount)?;
                nonempty(&pair.flange)?;
            }
            if let Some(index) = r.order_requires {
                let req = order
                    .and_then(|o| o.requires.get(index))
                    .ok_or("mounting order_requires index is out of range")?;
                if req.qty == 0 {
                    return Err("mounting required quantity must be positive".into());
                }
            }
        }
        Ok(())
    }
}

fn nonempty(value: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        Err("mounting identifiers must not be blank".into())
    } else {
        Ok(())
    }
}

fn positive(value: f64) -> Result<(), String> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err("mounting dimension must be finite and positive".into())
    }
}

fn feature<'a>(
    id: &'a str,
    position: &[f64; 2],
    tolerance: Option<f64>,
    ids: &mut std::collections::HashSet<&'a str>,
) -> Result<(), String> {
    nonempty(id)?;
    if !ids.insert(id)
        || !position.iter().all(|v| v.is_finite())
        || tolerance.is_some_and(|t| !t.is_finite() || t < 0.0)
    {
        return Err(
            "mounting features need unique ids, finite positions and nonnegative tolerances".into(),
        );
    }
    Ok(())
}

impl DimensionRange {
    pub fn validate(&self) -> Result<(), String> {
        if self.min.is_finite() && self.max.is_finite() && self.min >= 0.0 && self.max >= self.min {
            Ok(())
        } else {
            Err("mounting range must have finite 0 <= min <= max".into())
        }
    }
}

impl MountThread {
    fn validate(&self) -> Result<(), String> {
        positive(self.diameter_mm)?;
        if let Some(pitch) = self.pitch_mm {
            positive(pitch)?;
        }
        Ok(())
    }
}

impl MountingDocument {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != "1" {
            return Err("unsupported mounting document schema_version; expected 1".into());
        }
        nonempty(&self.revision)?;
        self.mounting.validate(&self.sources, self.order.as_ref())
    }

    /// Overlay declared faces while retaining the catalog's required parts.
    /// Source and order references are rebased, never resolved from a path again.
    pub fn apply_to(&self, base: &crate::CatalogMeta) -> crate::CatalogMeta {
        let mut result = base.clone();
        let source_offset = result.sources.len();
        result.sources.extend(self.sources.iter().cloned());
        let order_offset = result.order.as_ref().map_or(0, |o| o.requires.len());
        if let Some(order) = &self.order {
            if let Some(existing) = &mut result.order {
                existing.requires.extend(order.requires.iter().cloned());
            } else {
                result.order = Some(order.clone());
            }
        }
        let shift = |e: &mut Vec<MountEvidence>| {
            for e in e {
                e.source += source_offset;
            }
        };
        let spec = result.mounting.get_or_insert_with(MountingSpec::default);
        for face in &self.mounting.interfaces {
            let mut face = face.clone();
            shift(&mut face.evidence);
            if let Some(g) = &mut face.geometry {
                shift(&mut g.evidence);
            }
            if let Some(c) = &mut face.clearance {
                shift(&mut c.evidence);
            }
            for f in &mut face.fasteners {
                shift(&mut f.evidence);
            }
            if let Some(existing) = spec
                .interfaces
                .iter_mut()
                .find(|f| f.frame == face.frame && f.role == face.role)
            {
                *existing = face;
            } else {
                spec.interfaces.push(face);
            }
        }
        for req in &self.mounting.requirements {
            let mut req = req.clone();
            // Keep independently sourced obligations distinct even when an
            // author reuses the catalog requirement's display identifier.
            req.id = format!("document:{}", req.id);
            while spec.requirements.iter().any(|r| r.id == req.id) {
                req.id = format!("document:{}", req.id);
            }
            shift(&mut req.evidence);
            for alternative in &mut req.catalog_alternatives {
                shift(&mut alternative.evidence);
            }
            req.order_requires = req.order_requires.map(|i| i + order_offset);
            spec.requirements.push(req);
        }
        result
    }
}
