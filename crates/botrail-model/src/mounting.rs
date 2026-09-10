//! Declared mechanical interfaces and purchase data from a catalog manifest:
//! the mating faces a product names, the parts its installation documents
//! require between it and the robot, and where those statements come from.
//! Nothing here is inferred from geometry.

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

/// A mating face the manifest names. Drawing-level detail a manifest may
/// carry for it (hole patterns, fasteners, envelopes) is not read here.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
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
