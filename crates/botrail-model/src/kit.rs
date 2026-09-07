//! A purchase unit with an explicit physical assembly and scoped source claims.
use crate::mounting::{CatalogOrder, CatalogSource, MountEvidence};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct KitSpec {
    pub base: String,
    pub attachments: Vec<KitAttachment>,
    #[serde(default)]
    pub manufacturer_support: Vec<KitSupport>,
    pub representation: KitRepresentation,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct KitAttachment {
    pub package: String,
    pub alias: String,
    pub parent_frame: String,
    pub mount_frame: String,
    #[serde(default)]
    pub offset: KitOffset,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct KitOffset {
    #[serde(default)]
    pub xyz: [f64; 3],
    #[serde(default)]
    pub rpy: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct KitSupport {
    pub host: String,
    /// Only mechanical mounting is evaluated; electrical claims are separate.
    pub scope: KitScope,
    pub evidence: Vec<MountEvidence>,
    #[serde(default)]
    pub note: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum KitScope {
    MechanicalMounting,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct KitRepresentation {
    pub status: RepresentationStatus,
    #[serde(default)]
    pub evidence: Vec<MountEvidence>,
    pub note: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RepresentationStatus {
    Verified,
    Reference,
    Unknown,
}

impl KitSpec {
    pub fn validate(
        &self,
        sources: &[CatalogSource],
        order: Option<&CatalogOrder>,
    ) -> Result<(), String> {
        use std::collections::{BTreeMap, BTreeSet};
        fn slug(s: &str, dots: bool) -> bool {
            s.bytes()
                .next()
                .is_some_and(|b| b.is_ascii_lowercase() || b.is_ascii_digit())
                && s.bytes().all(|b| {
                    b.is_ascii_lowercase()
                        || b.is_ascii_digit()
                        || b"_-".contains(&b)
                        || (dots && b == b'.')
                })
        }
        fn id(s: &str) -> bool {
            let p: Vec<_> = s.split('/').collect();
            p.len() == 4
                && slug(p[0], false)
                && slug(p[1], false)
                && slug(p[2], true)
                && p[3].strip_prefix('r').is_some_and(|r| {
                    r.starts_with(['1', '2', '3', '4', '5', '6', '7', '8', '9'])
                        && r.bytes().all(|b| b.is_ascii_digit())
                })
        }
        let evidence = |refs: &[MountEvidence]| {
            refs.iter().all(|e| {
                !e.section.trim().is_empty()
                    && sources
                        .get(e.source)
                        .is_some_and(|s| !s.url.trim().is_empty())
            })
        };
        if !id(&self.base) || self.attachments.is_empty() {
            return Err("kit needs a full base catalog ID and attachments".into());
        }
        let mut expected = BTreeMap::from([(self.base.as_str(), 1u64)]);
        let mut aliases = BTreeSet::new();
        for a in &self.attachments {
            if !id(&a.package)
                || !slug(&a.alias, false)
                || !aliases.insert(&a.alias)
                || a.parent_frame.is_empty()
                || a.mount_frame.is_empty()
                || !a
                    .offset
                    .xyz
                    .iter()
                    .chain(&a.offset.rpy)
                    .all(|v| v.is_finite())
            {
                return Err("invalid kit attachment: IDs, unique aliases, frames and finite offsets required".into());
            }
            *expected.entry(&a.package).or_default() += 1;
        }
        let Some(order) = order else {
            return Err("kit requires order".into());
        };
        if order.unit != "kit"
            || order
                .part_number
                .as_ref()
                .is_none_or(|s| s.trim().is_empty())
        {
            return Err("kit requires order.unit=kit and a manufacturer part_number".into());
        }
        let mut included = BTreeMap::new();
        for item in &order.includes {
            if item.qty == 0 {
                return Err("kit include quantity must be positive".into());
            }
            if let Some(id) = &item.catalog {
                *included.entry(id.as_str()).or_insert(0u64) += u64::from(item.qty);
            }
        }
        if expected != included {
            return Err("kit assembly must match order.includes catalog quantities".into());
        }
        for claim in &self.manufacturer_support {
            if !id(&claim.host)
                || claim.evidence.is_empty()
                || !evidence(&claim.evidence)
                || !claim
                    .evidence
                    .iter()
                    .any(|e| sources[e.source].kind == "manufacturer_datasheet")
            {
                return Err(
                    "kit support requires an exact host and manufacturer documentary evidence"
                        .into(),
                );
            }
        }
        if self.representation.note.trim().is_empty()
            || !evidence(&self.representation.evidence)
            || (self.representation.status == RepresentationStatus::Verified
                && self.representation.evidence.is_empty())
        {
            return Err(
                "kit representation needs a note and valid evidence (required when verified)"
                    .into(),
            );
        }
        Ok(())
    }
}
