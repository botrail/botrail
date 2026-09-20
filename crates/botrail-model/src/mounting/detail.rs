//! Drawing vocabulary shared with catalog-builder/schema/mounting.py.
//! Lengths are millimetres; ranges are bounds, never nominal substitutes.

use super::*;
use std::collections::HashSet;

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

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct FastenerRules {
    #[serde(default)]
    pub thread: Option<MountThread>,
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
pub struct MountHole {
    pub id: String,
    pub position_mm: [f64; 2],
    pub kind: HoleKind,
    #[serde(default)]
    pub position_tolerance_mm: Option<f64>,
    #[serde(default)]
    pub diameter_mm: Option<DimensionRange>,
    #[serde(default)]
    pub thread: Option<MountThread>,
    /// Usable tip depth from the mating plane, including bottom allowance.
    #[serde(default)]
    pub depth_mm: Option<DimensionRange>,
    /// Unthreaded lead. Only this omitted dimension means zero by contract.
    #[serde(default)]
    pub thread_start_mm: Option<DimensionRange>,
    /// Bearing surface to mating face; excludes washers.
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
    /// Projection height or usable recess depth.
    #[serde(default)]
    pub depth_mm: Option<DimensionRange>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountGeometry {
    pub normal_z: i8,
    #[serde(default)]
    pub frame_verified: bool,
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
    pub holes: Vec<String>,
    pub thread: MountThread,
    pub head_standard: String,
    pub property_class: String,
    pub length_mm: DimensionRange,
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

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct MountClearance {
    #[serde(default)]
    pub frame_verified: bool,
    #[serde(default)]
    pub complete: bool,
    #[serde(default)]
    pub solids: Vec<MountBox>,
    #[serde(default)]
    pub access: Vec<MountBox>,
    #[serde(default)]
    pub evidence: Vec<MountEvidence>,
}

fn nonnegative(v: f64) -> Result<(), String> {
    if v.is_finite() && v >= 0.0 {
        Ok(())
    } else {
        Err("mounting dimensions must be finite and nonnegative".into())
    }
}

fn positive(v: f64) -> Result<(), String> {
    nonnegative(v)?;
    if v > 0.0 {
        Ok(())
    } else {
        Err("mounting dimension must be positive".into())
    }
}

impl DimensionRange {
    fn validate(&self) -> Result<(), String> {
        nonnegative(self.min)?;
        nonnegative(self.max)?;
        if self.min <= self.max {
            Ok(())
        } else {
            Err("mounting dimension bounds must be ordered".into())
        }
    }
}

impl MountThread {
    fn validate(&self) -> Result<(), String> {
        positive(self.diameter_mm)?;
        self.pitch_mm.map_or(Ok(()), positive)
    }
}

impl MountInterface {
    pub(super) fn validate_details(
        &self,
        evidence: &impl Fn(&[MountEvidence]) -> Result<(), String>,
    ) -> Result<(), String> {
        if let Some(g) = &self.geometry {
            if ![-1, 1].contains(&g.normal_z) {
                return Err("mounting normal_z must be -1 or 1".into());
            }
            evidence(&g.evidence)?;
            let mut ids = HashSet::new();
            for h in &g.holes {
                feature(&h.id, &h.position_mm, h.position_tolerance_mm, &mut ids)?;
                for d in [h.diameter_mm, h.depth_mm, h.thread_start_mm, h.grip_mm]
                    .into_iter()
                    .flatten()
                {
                    d.validate()?;
                }
                if h.kind == HoleKind::Clearance
                    && (h.thread.is_some() || h.thread_start_mm.is_some())
                {
                    return Err("mounting clearance holes cannot declare a thread".into());
                }
                if let Some(thread) = &h.thread {
                    thread.validate()?;
                }
                if let Some(r) = &h.fastener_rules {
                    if let Some(thread) = &r.thread {
                        thread.validate()?;
                    }
                    for name in [&r.head_standard, &r.property_class].into_iter().flatten() {
                        nonempty(name)?;
                    }
                    if let Some(d) = r.length_mm {
                        d.validate()?;
                        positive(d.min)?;
                    }
                    if let Some(d) = r.torque_nm {
                        d.validate()?;
                    }
                    if let Some(d) = r.min_engagement_mm {
                        positive(d)?;
                    }
                }
            }
            for f in &g.locators {
                feature(&f.id, &f.position_mm, f.position_tolerance_mm, &mut ids)?;
                for d in [f.diameter_mm, f.depth_mm].into_iter().flatten() {
                    d.validate()?;
                }
            }
        }
        let mut selected = HashSet::new();
        for f in &self.fasteners {
            evidence(&f.evidence)?;
            f.thread.validate()?;
            nonempty(&f.head_standard)?;
            nonempty(&f.property_class)?;
            for d in [f.length_mm, f.washer_mm, f.torque_nm] {
                d.validate()?;
            }
            positive(f.length_mm.min)?;
            if f.holes.is_empty() {
                return Err("mounting fastener needs a clearance hole".into());
            }
            for id in &f.holes {
                if !selected.insert(id)
                    || !self.geometry.as_ref().is_some_and(|g| {
                        g.holes
                            .iter()
                            .any(|h| h.id == *id && h.kind == HoleKind::Clearance)
                    })
                {
                    return Err(
                        "mounting fasteners must reference distinct declared clearance holes"
                            .into(),
                    );
                }
            }
        }
        if let Some(c) = &self.clearance {
            evidence(&c.evidence)?;
            let mut ids = HashSet::new();
            for b in c.solids.iter().chain(&c.access) {
                nonempty(&b.id)?;
                if !ids.insert(&b.id) || !b.center_mm.iter().all(|v| v.is_finite()) {
                    return Err("mounting boxes need unique ids and finite centers".into());
                }
                for v in b.size_mm {
                    positive(v)?;
                }
            }
            if c.complete && c.solids.is_empty() {
                return Err("complete clearance requires conservative solid envelopes".into());
            }
        }
        Ok(())
    }
}

fn feature<'a>(
    id: &'a str,
    xy: &[f64; 2],
    tolerance: Option<f64>,
    ids: &mut HashSet<&'a str>,
) -> Result<(), String> {
    nonempty(id)?;
    if !ids.insert(id) || !xy.iter().all(|v| v.is_finite()) {
        return Err("mounting features need unique ids and finite positions".into());
    }
    tolerance.map_or(Ok(()), nonnegative)
}
