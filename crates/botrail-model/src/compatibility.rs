//! Manufacturer compatibility statements carried by a catalog manifest.
//! `mounts` / `requires_adapter` are host lists, `programs` are ecosystem
//! listings (UR+, FANUC CRX plug-ins, …); anything else the manifest says
//! rides along untouched in `extra`.
use crate::mounting::{CatalogSource, MountEvidence};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Compatibility {
    #[serde(default)]
    pub mounts: Vec<String>,
    #[serde(default)]
    pub requires_adapter: Vec<String>,
    #[serde(default)]
    pub programs: Vec<ProgramListing>,
    #[serde(flatten)]
    pub extra: BTreeMap<String, Value>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct ProgramListing {
    pub name: String,
    pub status: ListingStatus,
    pub evidence: Vec<MountEvidence>,
    #[serde(default)]
    pub note: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum ListingStatus {
    Listed,
    Unknown,
}

impl Compatibility {
    /// A program listing needs a name and a cited manufacturer source.
    pub fn validate(&self, sources: &[CatalogSource]) -> Result<(), String> {
        let evidence = |refs: &[MountEvidence]| {
            !refs.is_empty()
                && refs.iter().all(|e| {
                    !e.section.trim().is_empty()
                        && sources
                            .get(e.source)
                            .is_some_and(|s| !s.url.trim().is_empty())
                })
                && refs.iter().any(|e| {
                    sources
                        .get(e.source)
                        .is_some_and(|s| s.kind == "manufacturer_datasheet")
                })
        };
        if self
            .programs
            .iter()
            .any(|p| p.name.trim().is_empty() || !evidence(&p.evidence))
        {
            return Err("program listing requires name and documentary evidence".into());
        }
        Ok(())
    }
}
